// AHC073 A - Circles of Inversion
//
// 핵심 관찰
//  1) 텔레포트 (T -> 2C - T) 는 각 좌표의 홀짝을 보존한다. 걷기만 홀짝을 바꾼다.
//  2) Q = T + 2*(원 좌표 합) 은 텔레포트로 변하지 않고 걷기로만 변한다.
//     따라서 원들의 무게중심은 (Q - T)/6 으로 묶여 있다. 걷기로 Q 가 치우치면 원들이 한쪽으로 몰린다.
//  3) 한 구간의 조작열은 "어느 원을 쓰는가"의 패턴 + 각 단계의 걷기 벡터로 표현된다.
//     패턴이 정해지면 모든 위치는 걷기 벡터의 아핀 함수이고, 계수는 홀수(1, 3, 5, 11, ...)다.
//     -> 패턴을 DFS 로 열거하고 걷기는 증폭 계수를 쓰는 동전 문제 + 시뮬레이션 클램프로 정한다.
//     또 패턴이 정해지면 x, y 축은 완전히 독립이라 축별 분기 한정으로 정확히 풀 수 있다.
//  4) 원 c 는 출발점 T 가 축마다 [2c-127, 2c] 안일 때만 쓸 수 있고, 쓰면 새 원이 T 에 생긴다.
//     가장자리 근처의 원은 같은 가장자리 근처에서만 쓸 수 있어 사실상 영구히 죽는다.
//     -> 구간 끝까지 살아남는 원은 중앙 영역 [40,87]^2 (완화 후보는 [28,99]^2) 에 두고,
//        그 밖의 원은 "앞으로 그 원을 거울로 깨끗이 쓸 목표가 올 때까지" 구간당 벌점을 준다.
//  5) 준비 계획: 다음 구간 출발점 D (목적지를 중앙 영역으로 당긴 점) 와 그 다음 목적지 G 의 중점에
//     원이 있으면 그 구간은 (D 까지 걷기) + 텔레포트 1회로 끝난다. 그래서 앞으로 몇 구간의
//     중점 근처에 출발점(= 새 원)을 두는 계획을 축별 정확 해 + 부드러운 벌점으로 만들어 후보에 넣는다.
//  6) 구간 단위 빔 서치: 점수 = 지금까지 조작 수 + 탐욕 롤아웃(다음 몇 구간) 비용
//                              + Q 균형 벌점 + 가장자리 원 벌점.
#include <bits/stdc++.h>
using namespace std;

static const int N = 128, MAXT = 8, MAXS = MAXT + 1;
struct Expr { int bx, by; int c[MAXS]; };  // 위치 = (bx,by) + sum_j c[j] * (j단계 걷기)
struct P2 { int x, y; bool operator==(const P2& o) const { return x == o.x && y == o.y; } };

static inline int floorDiv(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }
static inline bool inG(int x, int y) { return (unsigned)x < (unsigned)N && (unsigned)y < (unsigned)N; }

// 파라미터
const int BEAM_W = 16;        // 빔 폭
const int LEG_TMAX = 5;       // 한 구간 최대 텔레포트 수
const int LEG_SLACK = 10;     // 최적 + SLACK 이내 계획을 후보로
const int LOOK_TMAX = 4;      // 롤아웃 구간의 최대 텔레포트 수
const int ROLLOUT = 5;        // 롤아웃 구간 수
const double Q_WEIGHT = 0.2;  // Q 균형 벌점
const double Q_TARGET = 444.5;
const int SURV_MAIN = 40;     // 구간 끝까지 살아남는 원: 가장자리에서 이 거리 이상
const int SURV_RELAX = 28;    // 완화 후보
const int END_RELAX = 8;      // 마지막 몇 구간은 생존 원 제약 해제
const int SEMI_K = 8;         // 가장자리 원 벌점: 이만큼 앞의 목표까지 본다
const int SETUP_TMAX = 2;     // 준비 계획의 최대 텔레포트 수
const int SETUP_SLACK = 40;   // 준비 계획은 최적 + 이만큼까지
const int SETUP_AHEAD = 6;    // 몇 구간 앞까지 거울 위치를 준비하나
const int SETUP_MIN = 20;     // 이보다 짧은 구간은 준비하지 않음
const int SETUP_W = 2;        // 준비 원 위치 오차 벌점 (축별 |P - X| 당)
const int SETUP_CAP = 12;     // 축별 오차 허용 한도
const double TIME_LIMIT = 1.75;

int SURV = SURV_MAIN;

// ---------- 동전 문제: sum n_i * mags[i] = d 에서 sum |n_i| 최소 (mags 내림차순, 마지막 1) ----------
int cBest, cCur[MAXS], cAlloc[MAXS];
void coinRec(int d, const int* mags, int m, int idx, int acc) {
    if (acc >= cBest) return;
    if (idx == m - 1) {
        int c = acc + abs(d);
        if (c < cBest) { cBest = c; cCur[idx] = d; memcpy(cAlloc, cCur, sizeof(int) * m); }
        return;
    }
    int v = mags[idx];
    if (acc + abs(d) / v >= cBest) return;
    int q = floorDiv(d, v);
    for (int x = q; x <= q + 1; x++) { cCur[idx] = x; coinRec(d - x * v, mags, m, idx + 1, acc + abs(x)); }
}
static inline int coinLB(int d, const int* mags, int m, int cap) { cBest = cap; coinRec(d, mags, m, 0, 0); return cBest; }

// ---------- 패턴 표 (축별 정확 해용) ----------
struct PatInfo {
    int k; int ch[MAXT];
    bool surv[MAXT];            // j번째 출발점의 원이 구간 끝까지 남는가
    int mags[MAXS][MAXS], nm[MAXS];  // j단계 이후 걷기 계수의 서로 다른 절댓값 (내림차순)
};
vector<PatInfo> pats;
void buildPats(int maxK) {
    for (int k = 1; k <= maxK; k++) {
        int tot = 1; for (int t = 0; t < k; t++) tot *= 3;
        for (int m = 0; m < tot; m++) {
            PatInfo P; P.k = k; int mm = m; for (int t = 0; t < k; t++) { P.ch[t] = mm % 3; mm /= 3; }
            for (int j = 0; j < k; j++) { P.surv[j] = true; for (int jj = j + 1; jj < k; jj++) if (P.ch[jj] == P.ch[j]) P.surv[j] = false; }
            int x[MAXS] = {0}, C[3][MAXS]; memset(C, 0, sizeof(C)); x[0] = 1;
            for (int j = 0; j < k; j++) {
                int s = P.ch[j]; int nx[MAXS];
                for (int t = 0; t < MAXS; t++) nx[t] = 2 * C[s][t] - x[t];
                nx[j + 1] = 1;
                memcpy(C[s], x, sizeof(x)); memcpy(x, nx, sizeof(nx));
            }
            for (int j = 0; j <= k; j++) {
                int c = 0;
                for (int t = j; t <= k; t++) { int v = abs(x[t]); bool f = false; for (int q = 0; q < c; q++) if (P.mags[j][q] == v) f = true; if (!f) P.mags[j][c++] = v; }
                sort(P.mags[j], P.mags[j] + c, greater<int>()); P.nm[j] = c;
            }
            pats.push_back(P);
        }
    }
}

// 한 축에서 패턴 고정 시 출발점 P_j 를 분기 한정으로 탐색. 하한 = 남은 패턴을 걷기 0 으로 진행했을 때의
// 오차를 동전 문제로. softStage 단계의 출발점은 softX 에서 멀어질수록 softW 배 벌점.
struct AxisSolver {
    const PatInfo* P; int lo0[MAXT], hi0[MAXT]; int target, best; int sol[MAXS], cur[MAXS];
    int softStage = -1, softX = 0, softW = 2;
    int f0(int j, int pos, const int* cc) const {
        int c[3] = {cc[0], cc[1], cc[2]};
        for (int t = j; t < P->k; t++) { int s = P->ch[t]; int l = 2 * c[s] - pos; c[s] = pos; pos = l; }
        return pos;
    }
    void rec(int j, int pos, int* cc, int acc) {
        if (acc >= best) return;
        int k = P->k;
        if (j == k) { int c = acc + abs(target - pos); if (c < best) { best = c; cur[k] = target - pos; memcpy(sol, cur, sizeof(int) * (k + 1)); } return; }
        { int F = f0(j, pos, cc); int lb = coinLB(target - F, P->mags[j], P->nm[j], best - acc); if (acc + lb >= best) return; }
        int s = P->ch[j];
        int lo = max(lo0[j], 2 * cc[s] - (N - 1)), hi = min(hi0[j], 2 * cc[s]);
        if (lo > hi) return;
        bool soft = j == softStage;
        int start = soft ? min(max(softX, lo), hi) : min(max(pos, lo), hi);
        int base = soft ? abs(pos - softX) + abs(start - softX) : abs(start - pos);
        for (int d = 0;; d++) {
            if (acc + base + d >= best) break;
            bool any = false;
            for (int sg = -1; sg <= 1; sg += 2) {
                if (d == 0 && sg == 1) continue;
                int Pp = start + sg * d;
                if (Pp < lo || Pp > hi) continue;
                any = true;
                int a = acc + abs(Pp - pos) + (soft ? softW * abs(Pp - softX) : 0);
                if (a >= best) continue;
                int old = cc[s]; int l = 2 * old - Pp; cc[s] = Pp; cur[j] = Pp - pos;
                rec(j + 1, l, cc, a);
                cc[s] = old;
            }
            if (!any) break;
        }
    }
    int solve(int s0, const int* c0, int tgt, int bound) {
        target = tgt; best = bound;
        int cc[3] = {c0[0], c0[1], c0[2]};
        rec(0, s0, cc, 0);
        return best;
    }
};

// ---------- 한 구간 계획 ----------
struct Plan {
    int cost, k;            // 총 조작 수, 텔레포트 수
    int8_t ch[MAXT];        // j번째 텔레포트에 쓰는 원 슬롯
    int wx[MAXS], wy[MAXS]; // 단계별 걷기 (j < k: j번째 텔레포트 직전, k: 마지막)
    int8_t ord[MAXS];       // 걷기 순서 (0: 행 먼저, 1: 열 먼저)
    P2 endC[3];             // 구간 종료 시 원 위치
};

struct LegSearch {
    P2 S, G; P2 circ0[3];
    Expr X[MAXS], C[MAXS][3];
    int8_t ch[MAXT];
    int tmax, best, slack;
    bool collect;
    vector<Plan> plans;

    // 자유 걷기(fx,fy)를 적용하며 시뮬레이션. 점프 착지가 격자 밖이거나 살아남을 출발점이 중앙 영역 밖이면
    // 최소로 걸어서 클램프. 마지막엔 목표까지 걷기. 비용(초과 시 INT_MAX) 반환.
    int finish(int k, const int* fx, const int* fy, int lim, Plan* out) {
        int px = S.x, py = S.y;
        P2 cc[3] = {circ0[0], circ0[1], circ0[2]};
        int cost = k;
        Plan tmp; Plan* pl = out ? out : &tmp;
        for (int j = 0; j < k; j++) {
            int nx = px + fx[j], ny = py + fy[j];
            int s = ch[j];
            int sf = 0;
            if (SURV > 0) {
                bool survive = true;  // 이후 같은 슬롯을 쓰지 않으면 이 출발점의 원은 끝까지 남는다
                for (int jj = j + 1; jj < k; jj++) if (ch[jj] == ch[j]) { survive = false; break; }
                if (survive) sf = SURV;
            }
            int lox = max(sf, 2 * cc[s].x - (N - 1)), hix = min(N - 1 - sf, 2 * cc[s].x);
            int loy = max(sf, 2 * cc[s].y - (N - 1)), hiy = min(N - 1 - sf, 2 * cc[s].y);
            if (lox > hix || loy > hiy) return INT_MAX;
            nx = min(max(nx, lox), hix); ny = min(max(ny, loy), hiy);
            pl->wx[j] = nx - px; pl->wy[j] = ny - py;
            cost += abs(pl->wx[j]) + abs(pl->wy[j]);
            if (cost > lim) return INT_MAX;
            px = nx; py = ny;
            int lx = 2 * cc[s].x - px, ly = 2 * cc[s].y - py;
            cc[s] = {px, py};
            px = lx; py = ly;
        }
        pl->wx[k] = G.x - px; pl->wy[k] = G.y - py;
        cost += abs(pl->wx[k]) + abs(pl->wy[k]);
        if (cost > lim) return INT_MAX;
        pl->cost = cost; pl->k = k;
        memcpy(pl->ch, ch, sizeof(ch));
        if (!validate(k, pl)) return INT_MAX;
        return cost;
    }

    // 걷기 경로가 목표를 일찍 밟지 않는지 등 확인, 걷기 순서와 종료 원 위치 결정
    bool validate(int k, Plan* pl) {
        int px = S.x, py = S.y;
        P2 cc[3] = {circ0[0], circ0[1], circ0[2]};
        for (int j = 0; j <= k; j++) {
            int nx = px + pl->wx[j], ny = py + pl->wy[j];
            int ordv = 0;
            if (j < k && (nx != px || ny != py)) {
                auto hits = [&](int o) {
                    if (o == 0) {
                        if (G.y == py && min(px, nx) <= G.x && G.x <= max(px, nx)) return true;
                        if (G.x == nx && min(py, ny) <= G.y && G.y <= max(py, ny)) return true;
                    } else {
                        if (G.x == px && min(py, ny) <= G.y && G.y <= max(py, ny)) return true;
                        if (G.y == ny && min(px, nx) <= G.x && G.x <= max(px, nx)) return true;
                    }
                    return false;
                };
                if (!hits(0)) ordv = 0; else if (!hits(1)) ordv = 1; else return false;
            }
            pl->ord[j] = ordv;
            px = nx; py = ny;
            if (j < k) {
                int s = pl->ch[j];
                int lx = 2 * cc[s].x - px, ly = 2 * cc[s].y - py;
                if (!inG(lx, ly)) return false;
                cc[s] = {px, py};
                px = lx; py = ly;
                if (px == G.x && py == G.y && !(j == k - 1 && pl->wx[k] == 0 && pl->wy[k] == 0)) return false;
            }
        }
        if (px != G.x || py != G.y) return false;
        for (int i = 0; i < 3; i++) pl->endC[i] = cc[i];
        return true;
    }

    void consider(int k, const int* fx, const int* fy) {
        int lim = best + slack;
        if (collect) {
            Plan pl;
            int c = finish(k, fx, fy, lim, &pl);
            if (c <= lim) { plans.push_back(pl); if (c < best) best = c; }
        } else {
            int c = finish(k, fx, fy, best - 1, nullptr);
            if (c < best) best = c;
        }
    }

    void dfs(int k) {
        Expr& x = X[k];
        int dx = G.x - x.bx, dy = G.y - x.by;
        int p = (dx & 1) + (dy & 1);  // 홀짝 불일치 수 = 필요한 최소 걷기
        int maxc = 1;
        for (int j = 0; j < k; j++) maxc = max(maxc, abs(x.c[j]));
        int outd = max(0, -x.bx) + max(0, x.bx - (N - 1)) + max(0, -x.by) + max(0, x.by - (N - 1));
        int lbw = (outd + maxc - 1) / maxc;
        int lim = best + slack;
        if (k + max(p, lbw) <= lim) {
            int fx[MAXS] = {0}, fy[MAXS] = {0};
            consider(k, fx, fy);  // 자유 걷기 없이 (강제 클램프 + 마지막 걷기)
            // 증폭 계수를 쓰는 동전 해
            int mags[MAXS], magStage[MAXS], m = 0;
            for (int j = k; j >= 0; j--) {
                int v = abs(x.c[j]);
                bool f = false;
                for (int t = 0; t < m; t++) if (mags[t] == v) f = true;
                if (!f) { mags[m] = v; magStage[m] = j; m++; }
            }
            if (m > 1) {
                for (int a = 0; a < m; a++) for (int b = a + 1; b < m; b++) if (mags[b] > mags[a]) { swap(mags[a], mags[b]); swap(magStage[a], magStage[b]); }
                int cap = best + slack - k;
                cBest = cap + 1; coinRec(dx, mags, m, 0, 0);
                int cx = cBest;
                if (cx <= cap) {
                    int ax[MAXS]; memcpy(ax, cAlloc, sizeof(int) * m);
                    cBest = cap - cx + 1; coinRec(dy, mags, m, 0, 0);
                    int cy = cBest;
                    if (cx + cy <= cap) {
                        int ay[MAXS]; memcpy(ay, cAlloc, sizeof(int) * m);
                        bool any = false;
                        for (int t = 0; t < m; t++) {
                            if (mags[t] == 1) continue;
                            int st = magStage[t], sg = x.c[st] > 0 ? 1 : -1;
                            fx[st] += ax[t] * sg; fy[st] += ay[t] * sg;
                            if (ax[t] || ay[t]) any = true;
                        }
                        if (any) consider(k, fx, fy);
                    }
                }
            }
        }
        if (k == tmax || k + 1 + max(p, lbw) > best + slack) return;
        for (int i = 0; i < 3; i++) {
            bool dup = false;
            for (int j = 0; j < i; j++) if (!memcmp(&C[k][i], &C[k][j], sizeof(Expr))) dup = true;
            if (dup) continue;
            const Expr& c = C[k][i];
            Expr& nx = X[k + 1];
            nx.bx = 2 * c.bx - x.bx; nx.by = 2 * c.by - x.by;
            for (int j = 0; j < MAXS; j++) nx.c[j] = 2 * c.c[j] - x.c[j];
            nx.c[k + 1] = 1;
            memcpy(C[k + 1], C[k], sizeof(C[k]));
            C[k + 1][i] = x;  // 쓴 원은 출발점으로 이동
            ch[k] = i;
            dfs(k + 1);
        }
    }

    // 준비 계획: 살아남는 출발점 하나를 X 근처에 둔다 (축별 정확 해, 오차 |P-X| 에 SETUP_W 배 벌점).
    // 그 출발점은 중앙 영역 제약에서 면제. 실제 비용이 lim 이내인 계획만 out 에 추가.
    void setupPass(P2 X, int lim, vector<Plan>& out) {
        AxisSolver A;
        int cx[3] = {circ0[0].x, circ0[1].x, circ0[2].x}, cy[3] = {circ0[0].y, circ0[1].y, circ0[2].y};
        const int extra = SETUP_W * SETUP_CAP;
        for (const PatInfo& P : pats) {
            int k = P.k;
            if (k > lim) continue;
            for (int js = 0; js < k; js++) {
                if (!P.surv[js]) continue;
                A.P = &P; A.softStage = js; A.softW = SETUP_W;
                for (int j = 0; j < k; j++) {
                    int sf = (P.surv[j] && SURV > 0 && j != js) ? SURV : 0;
                    A.lo0[j] = sf; A.hi0[j] = N - 1 - sf;
                }
                A.softX = X.x;
                int bx = A.solve(S.x, cx, G.x, lim - k + 1 + extra);
                if (bx >= lim - k + 1 + extra) continue;
                int solx[MAXS]; memcpy(solx, A.sol, sizeof(int) * (k + 1));
                int tx = 0; for (int j = 0; j <= k; j++) tx += abs(solx[j]);
                if (k + tx > lim) continue;
                A.softX = X.y;
                int by = A.solve(S.y, cy, G.y, lim - k - tx + 1 + extra);
                if (by >= lim - k - tx + 1 + extra) continue;
                int ty = 0; for (int j = 0; j <= k; j++) ty += abs(A.sol[j]);
                if (k + tx + ty > lim) continue;
                Plan pl; pl.k = k; pl.cost = k + tx + ty;
                for (int j = 0; j < k; j++) pl.ch[j] = P.ch[j];
                for (int j = 0; j <= k; j++) { pl.wx[j] = solx[j]; pl.wy[j] = A.sol[j]; }
                if (!validate(k, &pl)) continue;
                out.push_back(pl);
            }
        }
    }

    void init(P2 s, const P2* circ, P2 g) {
        S = s; G = g;
        memset(X, 0, sizeof(X)); memset(C, 0, sizeof(C));
        X[0].bx = s.x; X[0].by = s.y; X[0].c[0] = 1;
        for (int i = 0; i < 3; i++) { circ0[i] = circ[i]; C[0][i].bx = circ[i].x; C[0][i].by = circ[i].y; }
    }
    void collectPlans(P2 s, const P2* circ, P2 g, int tm, int sl) {
        init(s, circ, g);
        tmax = tm; slack = sl; collect = true; plans.clear();
        best = abs(g.x - s.x) + abs(g.y - s.y) + 1;
        dfs(0);
        int b = best;
        plans.erase(remove_if(plans.begin(), plans.end(), [&](const Plan& p) { return p.cost > b + sl; }), plans.end());
    }
    const Plan& bestPlan() const {
        const Plan* bp = &plans[0];
        for (auto& q : plans) if (q.cost < bp->cost) bp = &q;
        return *bp;
    }
};

void emitPlan(const Plan& pl, P2& pos, P2* circ, vector<string>& ops) {
    for (int j = 0; j <= pl.k; j++) {
        int wx = pl.wx[j], wy = pl.wy[j];
        auto mx = [&]() { while (wx > 0) { ops.push_back("D"); wx--; pos.x++; } while (wx < 0) { ops.push_back("U"); wx++; pos.x--; } };
        auto my = [&]() { while (wy > 0) { ops.push_back("R"); wy--; pos.y++; } while (wy < 0) { ops.push_back("L"); wy++; pos.y--; } };
        if (pl.ord[j] == 0) { mx(); my(); } else { my(); mx(); }
        if (j < pl.k) {
            int s = pl.ch[j];
            ops.push_back("J " + to_string(circ[s].x) + " " + to_string(circ[s].y));
            P2 np = {2 * circ[s].x - pos.x, 2 * circ[s].y - pos.y};
            circ[s] = pos; pos = np;
        }
    }
}

struct Node { P2 circ[3]; int cost; int parent; Plan plan; int score; };

static inline uint64_t keyOf(const P2* c) {
    int v[3]; for (int i = 0; i < 3; i++) v[i] = c[i].x * N + c[i].y;
    sort(v, v + 3);
    return ((uint64_t)v[0] << 28) | ((uint64_t)v[1] << 14) | (uint64_t)v[2];
}

// 가장자리에서 d 떨어진 원 하나의 구간당 손실 (원을 주입하는 실험으로 보정한 값)
static double edgeRate(double d) {
    static const double xs[] = {0, 8, 16, 24, 32, 40}, ys[] = {5.25, 5.25, 2.48, 0.62, 0.36, 0};
    if (d >= 40) return 0.0;
    for (int t = 0; t < 5; t++) if (d <= xs[t + 1]) return ys[t] + (ys[t + 1] - ys[t]) * (d - xs[t]) / (xs[t + 1] - xs[t]);
    return 0.0;
}

int main() {
    auto t0 = chrono::steady_clock::now();
    auto elapsed = [&]() { return chrono::duration<double>(chrono::steady_clock::now() - t0).count(); };

    int n, M, K, TT;
    if (scanf("%d %d %d %d", &n, &M, &K, &TT) != 4) return 0;
    P2 Sst;
    if (scanf("%d %d", &Sst.x, &Sst.y) != 2) return 0;
    vector<P2> Gs(M);
    for (auto& g : Gs) if (scanf("%d %d", &g.x, &g.y) != 2) return 0;
    buildPats(SETUP_TMAX);

    // 초기 원: 하나는 첫 구간의 중점, 나머지 둘은 Q = s + 2*합 이 균형값(444.5) 근처가 되도록
    P2 init[3];
    init[0] = {(Sst.x + Gs[0].x) / 2, (Sst.y + Gs[0].y) / 2};
    {
        int rx = (int)((Q_TARGET - Sst.x) / 2) - init[0].x, ry = (int)((Q_TARGET - Sst.y) / 2) - init[0].y;
        init[1] = {min(N - 1, max(0, rx / 2)), min(N - 1, max(0, ry / 2))};
        init[2] = {min(N - 1, max(0, rx - init[1].x)), min(N - 1, max(0, ry - init[1].y))};
    }

    // 원 e 를 남겼을 때의 벌점: 가장자리 근처면 "e 를 거울로 써서 중앙에서 깨끗이 착지할 목표"가
    // 나올 때까지 구간당 edgeRate 만큼 (없으면 남은 구간 전부)
    auto edgePenalty = [&](P2 e, int r) {
        int ex = min(e.x, N - 1 - e.x), ey = min(e.y, N - 1 - e.y);
        int e1 = min(ex, ey), e2 = max(ex, ey);
        double rt = edgeRate(e1) * (e2 < SURV_MAIN ? 1.5 : 1.0);
        if (rt <= 0) return 0.0;
        int kUse = -1;
        for (int k = 1; k <= SEMI_K && r + k < M; k++) {
            P2 q = Gs[r + k];
            int ux = 2 * e.x - q.x, uy = 2 * e.y - q.y;
            if (ux >= SURV_MAIN && ux <= N - 1 - SURV_MAIN && uy >= SURV_MAIN && uy <= N - 1 - SURV_MAIN) { kUse = k; break; }
        }
        return rt * (kUse > 0 ? kUse : (M - r));
    };

    LegSearch L, H;
    vector<vector<Node>> levels(M + 1);
    Node root; for (int i = 0; i < 3; i++) root.circ[i] = init[i];
    root.cost = 0; root.parent = -1; root.score = 0;
    levels[0].push_back(root);
    int W = BEAM_W;
    long long statesDone = 0;
    vector<Plan> allp;
    for (int r = 0; r < M; r++) {
        // 시간 관리: 지금까지의 상태당 시간으로 남은 구간을 감당할 수 있는 빔 폭을 정한다
        double el = elapsed();
        if (r >= 5) {
            double perState = el / max(1LL, statesDone);
            W = max(1, min(BEAM_W, (int)((TIME_LIMIT - el) / (perState * (M - r)))));
        }
        statesDone += levels[r].size();
        P2 from = r == 0 ? Sst : Gs[r - 1];
        const int survNow = (M - r <= END_RELAX) ? 0 : SURV_MAIN;
        vector<Node> cand;
        for (int idx = 0; idx < (int)levels[r].size(); idx++) {
            const Node& nd = levels[r][idx];
            // 후보: 생존 원 제약 [40,87]^2 계획 + 완화 [28,99]^2 계획 + 준비 계획
            SURV = survNow;
            L.collectPlans(from, nd.circ, Gs[r], LEG_TMAX, LEG_SLACK);
            allp = L.plans;
            SURV = SURV_RELAX;
            L.collectPlans(from, nd.circ, Gs[r], LEG_TMAX, LEG_SLACK);
            allp.insert(allp.end(), L.plans.begin(), L.plans.end());
            SURV = survNow;
            if (survNow > 0) {
                int bestNow = INT_MAX; for (auto& q : allp) bestNow = min(bestNow, q.cost);
                L.init(from, nd.circ, Gs[r]);
                for (int ah = 1; ah <= SETUP_AHEAD && r + ah < M; ah++) {
                    // ah 구간 뒤의 구간: 출발점 D = 이전 목적지를 중앙 영역으로 당긴 점 (중점이 정수가 되게 홀짝 맞춤)
                    P2 g1 = Gs[r + ah - 1], g2 = Gs[r + ah];
                    auto clampPar = [&](int v, int t) { int p = min(max(v, SURV_MAIN), N - 1 - SURV_MAIN); if ((p + t) & 1) p += (p < 64 ? 1 : -1); return p; };
                    P2 D = {clampPar(g1.x, g2.x), clampPar(g1.y, g2.y)};
                    if (abs(g2.x - D.x) + abs(g2.y - D.y) < SETUP_MIN) continue;
                    L.setupPass(P2{(D.x + g2.x) / 2, (D.y + g2.y) / 2}, bestNow + SETUP_SLACK, allp);
                }
            }
            for (const Plan& pl : allp) {
                Node c; for (int i = 0; i < 3; i++) c.circ[i] = pl.endC[i];
                c.cost = nd.cost + pl.cost; c.parent = idx; c.plan = pl;
                // 탐욕 롤아웃: 다음 ROLLOUT 구간을 각각 최소 비용 계획으로 진행
                int h = 0;
                {
                    P2 cc[3] = {c.circ[0], c.circ[1], c.circ[2]}; P2 pp = Gs[r];
                    for (int t = 1; t <= ROLLOUT && r + t < M; t++) {
                        H.collectPlans(pp, cc, Gs[r + t], LOOK_TMAX, 0);
                        const Plan& bp = H.bestPlan();
                        h += bp.cost;
                        for (int i = 0; i < 3; i++) cc[i] = bp.endC[i];
                        pp = Gs[r + t];
                    }
                }
                double qx = Gs[r].x + 2.0 * (c.circ[0].x + c.circ[1].x + c.circ[2].x);
                double qy = Gs[r].y + 2.0 * (c.circ[0].y + c.circ[1].y + c.circ[2].y);
                double qd = fabs(qx - Q_TARGET) + fabs(qy - Q_TARGET);
                double pen = 0;
                for (int i = 0; i < 3; i++) pen += edgePenalty(c.circ[i], r);
                c.score = (int)((c.cost + h + Q_WEIGHT * qd + pen) * 16);
                cand.push_back(c);
            }
        }
        sort(cand.begin(), cand.end(), [](const Node& a, const Node& b) { return a.score != b.score ? a.score < b.score : a.cost < b.cost; });
        unordered_set<uint64_t> seen;
        for (auto& c : cand) {
            if ((int)levels[r + 1].size() >= W) break;
            if (!seen.insert(keyOf(c.circ)).second) continue;
            levels[r + 1].push_back(c);
        }
    }

    int bi = 0;
    for (int i = 0; i < (int)levels[M].size(); i++) if (levels[M][i].cost < levels[M][bi].cost) bi = i;
    vector<Plan> plans(M);
    for (int r = M, i = bi; r > 0; r--) { plans[r - 1] = levels[r][i].plan; i = levels[r][i].parent; }
    vector<string> ops; P2 pos = Sst; P2 circ[3] = {init[0], init[1], init[2]};
    for (int r = 0; r < M; r++) emitPlan(plans[r], pos, circ, ops);
    string out;
    for (int i = 0; i < 3; i++) out += to_string(init[i].x) + " " + to_string(init[i].y) + "\n";
    for (auto& s : ops) { out += s; out += '\n'; }
    fwrite(out.data(), 1, out.size(), stdout);
    fprintf(stderr, "A=%zu time=%.3f W=%d\n", ops.size(), elapsed(), W);
}
