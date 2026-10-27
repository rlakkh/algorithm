// AHC073 A - Circles of Inversion
//
// 핵심 관찰
//  1) 텔레포트 (T -> 2C - T) 는 각 좌표의 홀짝을 보존한다. 걷기만 홀짝을 바꾼다.
//  2) Q = T + 2*(원 좌표 합) 은 텔레포트로 변하지 않고 걷기로만 변한다.
//     따라서 원들의 무게중심은 (Q - T)/6 으로 묶여 있다. 걷기로 Q 가 치우치면 원들이 한쪽으로 몰린다.
//  3) 한 구간의 조작열은 "어느 원을 쓰는가"의 패턴 + 각 단계의 걷기 벡터로 표현된다.
//     패턴이 정해지면 모든 위치는 걷기 벡터의 아핀 함수이고, 계수는 홀수(1, 3, 5, 11, ...)다.
//     -> 패턴을 DFS 로 열거하고 걷기는 증폭 계수를 쓰는 동전 문제 + 시뮬레이션 클램프로 정한다.
//  4) 가장자리 근처에 생긴 원은 같은 가장자리 근처의 원으로만 교체될 수 있어 사실상 영구히 죽는다.
//     한 구간에서 끝까지 살아남는 원(마지막 출발점 등)은 중앙 영역에 두도록 강제한다.
//  5) 구간 단위 빔 서치: 점수 = 지금까지 조작 수 + 다음 구간 최소 비용 + Q 균형 벌점.
#include <bits/stdc++.h>
using namespace std;

static const int N = 128, MAXT = 8, MAXS = MAXT + 1;
struct Expr { int bx, by; int c[MAXS]; };  // 위치 = (bx,by) + sum_j c[j] * (j단계 걷기)
struct P2 { int x, y; bool operator==(const P2& o) const { return x == o.x && y == o.y; } };

static inline int floorDiv(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }
static inline bool inG(int x, int y) { return (unsigned)x < (unsigned)N && (unsigned)y < (unsigned)N; }

// 파라미터
const int BEAM_W = 8;        // 빔 폭
const int LEG_TMAX = 7;      // 한 구간 최대 텔레포트 수
const int LEG_SLACK = 10;    // 최적 + SLACK 이내 계획을 후보로
const int LOOK_TMAX = 5;     // 다음 구간 비용 추정 시 최대 텔레포트 수
const double Q_WEIGHT = 0.2; // Q 균형 벌점
const double Q_TARGET = 444.5;
const int SAFE_ALL = 24;     // 모든 출발점: 가장자리에서 이 거리 이상
const int SAFE_SURV = 40;    // 구간 끝까지 살아남는 원: 가장자리에서 이 거리 이상
const int END_RELAX = 8;     // 마지막 몇 구간은 생존 원 제약 해제
const double TIME_LIMIT = 1.7;

int SAFE = SAFE_ALL, SURV = SAFE_SURV;

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
    int tmax, best, slack, safe0 = 0;
    bool collect;
    vector<Plan> plans;

    // 자유 걷기(fx,fy)를 적용하며 시뮬레이션. 점프 착지가 격자 밖이거나 출발점이 안전 영역 밖이면
    // 최소로 걸어서 클램프. 마지막엔 목표까지 걷기. 비용(초과 시 INT_MAX) 반환.
    int finish(int k, const int* fx, const int* fy, int lim, Plan* out) {
        int px = S.x, py = S.y;
        P2 cc[3] = {circ0[0], circ0[1], circ0[2]};
        int cost = k;
        Plan tmp; Plan* pl = out ? out : &tmp;
        for (int j = 0; j < k; j++) {
            int nx = px + fx[j], ny = py + fy[j];
            int s = ch[j];
            int sf = j == 0 ? safe0 : SAFE;
            if (SURV > 0) {
                bool survive = true;  // 이후 같은 슬롯을 쓰지 않으면 이 출발점의 원은 끝까지 남는다
                for (int jj = j + 1; jj < k; jj++) if (ch[jj] == ch[j]) { survive = false; break; }
                if (survive) sf = max(sf, SURV);
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

    void init(P2 s, const P2* circ, P2 g) {
        S = s; G = g;
        memset(X, 0, sizeof(X)); memset(C, 0, sizeof(C));
        X[0].bx = s.x; X[0].by = s.y; X[0].c[0] = 1;
        for (int i = 0; i < 3; i++) { circ0[i] = circ[i]; C[0][i].bx = circ[i].x; C[0][i].by = circ[i].y; }
    }
    int bestCost(P2 s, const P2* circ, P2 g, int tm, int cap) {
        init(s, circ, g);
        safe0 = SAFE; tmax = tm; slack = 0; collect = false;
        best = min(cap, abs(g.x - s.x) + abs(g.y - s.y));
        dfs(0);
        return best;
    }
    void collectPlans(P2 s, const P2* circ, P2 g, int tm, int sl, int sf0 = -1) {
        init(s, circ, g);
        safe0 = sf0 < 0 ? SAFE : sf0;
        tmax = tm; slack = sl; collect = true; plans.clear();
        best = abs(g.x - s.x) + abs(g.y - s.y) + 1;
        dfs(0);
        int b = best;
        plans.erase(remove_if(plans.begin(), plans.end(), [&](const Plan& p) { return p.cost > b + sl; }), plans.end());
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

struct Node { P2 circ[3]; int cost; int parent; Plan plan; double score; };

static inline uint64_t keyOf(const P2* c) {
    int v[3]; for (int i = 0; i < 3; i++) v[i] = c[i].x * N + c[i].y;
    sort(v, v + 3);
    return ((uint64_t)v[0] << 28) | ((uint64_t)v[1] << 14) | (uint64_t)v[2];
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

    // 초기 원: 하나는 첫 구간의 중점, 나머지 둘은 Q = s + 2*합 이 균형값(444.5) 근처가 되도록
    P2 init[3];
    init[0] = {(Sst.x + Gs[0].x) / 2, (Sst.y + Gs[0].y) / 2};
    {
        int rx = (int)((Q_TARGET - Sst.x) / 2) - init[0].x, ry = (int)((Q_TARGET - Sst.y) / 2) - init[0].y;
        init[1] = {min(N - 1, max(0, rx / 2)), min(N - 1, max(0, ry / 2))};
        init[2] = {min(N - 1, max(0, rx - init[1].x)), min(N - 1, max(0, ry - init[1].y))};
    }

    LegSearch L, H;
    vector<vector<Node>> levels(M + 1);
    Node root; for (int i = 0; i < 3; i++) root.circ[i] = init[i];
    root.cost = 0; root.parent = -1; root.score = 0;
    levels[0].push_back(root);
    int W = BEAM_W;
    for (int r = 0; r < M; r++) {
        // 시간 관리: 남은 구간을 감안해 빔 폭 축소
        double el = elapsed();
        if (r > 0) {
            double perLeg = el / r;
            while (W > 1 && el + perLeg * (M - r) * 1.0 > TIME_LIMIT) { perLeg *= (double)(W - 1) / W; W--; }
        }
        P2 from = r == 0 ? Sst : Gs[r - 1];
        SURV = (M - r <= END_RELAX) ? 0 : SAFE_SURV;
        vector<Node> cand;
        for (int idx = 0; idx < (int)levels[r].size(); idx++) {
            const Node& nd = levels[r][idx];
            L.collectPlans(from, nd.circ, Gs[r], LEG_TMAX, LEG_SLACK);
            vector<Plan> allp = L.plans;
            // 첫 출발 전 더 안쪽으로 걸어 들어가는 변형
            L.collectPlans(from, nd.circ, Gs[r], LEG_TMAX, LEG_SLACK, SAFE_ALL + 4);
            allp.insert(allp.end(), L.plans.begin(), L.plans.end());
            for (const Plan& pl : allp) {
                Node c; for (int i = 0; i < 3; i++) c.circ[i] = pl.endC[i];
                c.cost = nd.cost + pl.cost; c.parent = idx; c.plan = pl;
                int h = r + 1 < M ? H.bestCost(Gs[r], c.circ, Gs[r + 1], LOOK_TMAX, 60) : 0;
                double qx = Gs[r].x + 2.0 * (c.circ[0].x + c.circ[1].x + c.circ[2].x);
                double qy = Gs[r].y + 2.0 * (c.circ[0].y + c.circ[1].y + c.circ[2].y);
                double qd = fabs(qx - Q_TARGET) + fabs(qy - Q_TARGET);
                c.score = c.cost + h + Q_WEIGHT * qd;
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
    fprintf(stderr, "A=%zu time=%.3f\n", ops.size(), elapsed());
}
