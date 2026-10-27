// AHC073 A - Circles of Inversion
//
// 모델: 각 구간(직전 목적지 a -> 다음 목적지 b)은 "걷기만" 하거나, 1~2개의 하위 구간으로 나눠
//   하위 구간마다 텔레포트 1회를 한다. 하위 구간 (a -> b) 의 텔레포트: a 에서 출발점 D 까지 걷고,
//   원 X 를 써서 2X - D 로 착지한 뒤 b 까지 걷는다. 비용 = |D - a| + 1 + |2X - D - b| (L1).
//   쓴 원 X 는 사라지고 D 에 새 원이 생긴다. 2회 텔레포트 구간은 경유점 m 으로 (a -> m), (m -> b) 로 나눈다.
//
// 핵심 관찰
//  1) 원 3개를 슬롯으로 보면, 하위 구간이 슬롯 j 를 쓰면 그 거울 원은 "슬롯 j 를 직전에 쓴 하위 구간의
//     출발점"(없으면 초기 원) 이고, 이번 출발점이 슬롯 j 의 새 원이 된다.
//     -> 하위 구간마다 쓸 슬롯(배정)과 경유점을 정하면, 슬롯마다 원 위치들이 사슬로 이어지고
//        서로 다른 슬롯은 완전히 독립이다. 게다가 x, y 축도 독립이다.
//  2) 따라서 배정이 주어지면 비용 최소의 출발점/원 위치는 슬롯별·축별 사슬 DP 로 정확히 구해진다.
//        F_k(D) = |D - a_k| + min_{X: 0 <= 2X-D <= 127} F_{k-1}(X) + |2X - D - b_k|,  F_0 = 0
//     (초기 원 위치도 자유롭게 고를 수 있으므로 F_0 = 0). 내부 min 은 X 를 2X <= D+b, 2X >= D+b 로
//     나누면 구간 최솟값 두 개가 되고, 구간 양 끝이 D 에 대해 단조라 단조 덱으로 O(128) 에 계산된다.
//  3) 전방 F 와 (끝에서부터 센) 후방 R 배열을 들고 있으면 한 구간의 배정/경유점 변경이나 두 슬롯의
//     꼬리 교환의 비용 변화를 O(128) 에 평가할 수 있고, 수락 시에도 바뀐 블록 앞뒤 배열은 재사용한다.
//     -> 배정·경유점에 대한 담금질(SA).
//  4) 원이 가장자리 근처에서 죽는 문제 등은 모두 DP 비용에 정확히 반영되므로 별도 휴리스틱이 필요 없다.
//     (목적지를 일찍 지나가도 방문 순서는 깨지지 않으므로 경로 검사는 필요 없다.)
#include <bits/stdc++.h>
using namespace std;

static const int N = 128, INF = 1000000000, MAXM = 160, MAXL = 2 * MAXM;
typedef array<int, N> Arr;

const double TIME_LIMIT = 1.75;      // 담금질 시간
const double TEMP0 = 20, TEMP1 = 0.5;
const int SPLIT_TRY = 15;            // 나뉘지 않은 구간에 대한 이동 중 2회 텔레포트로 나눠 보는 비율(%)

int M;
int Gx[MAXM + 1], Gy[MAXM + 1];       // Gx[0] = 시작, Gx[s+1] = s번째 목적지
int SA_[2][MAXL], SB_[2][MAXL];      // 하위 구간(id = 2s, 2s+1)의 시작/끝 좌표 (축별)
Arr ZERO;

// g[D] = |D-a| + min_{X: 0<=2X-D<=127} f[X] + |2X - D - b|
static void fwdStep(const Arr& f, int a, int b, Arr& g) {
    static int u[N], v[N], dq[N], p1[N];
    for (int X = 0; X < N; X++) { u[X] = f[X] - 2 * X; v[X] = f[X] + 2 * X; }
    // 2X <= D+b 쪽: X in [ceil(D/2), floor((D+b)/2)]
    int h = 0, t = 0, nx = 0;
    for (int D = 0; D < N; D++) {
        int L = (D + 1) >> 1, R = (D + b) >> 1;
        while (nx <= R) { while (t > h && u[dq[t - 1]] >= u[nx]) t--; dq[t++] = nx++; }
        while (t > h && dq[h] < L) h++;
        p1[D] = (t > h && L <= R) ? u[dq[h]] + D + b : INF;
    }
    // 2X >= D+b 쪽: X in [ceil((D+b)/2), floor((D+127)/2)]
    h = 0; t = 0; nx = 0;
    for (int D = 0; D < N; D++) {
        int L = (D + b + 1) >> 1, R = (D + N - 1) >> 1;
        while (nx <= R) { while (t > h && v[dq[t - 1]] >= v[nx]) t--; dq[t++] = nx++; }
        while (t > h && dq[h] < L) h++;
        int p2 = (t > h && L <= R) ? v[dq[h]] - D - b : INF;
        int m = min(p1[D], p2);
        g[D] = m >= INF ? INF : m + abs(D - a);
    }
}
// Bk[X] = min_{D in [2X-127, 2X] ∩ [0,127]} |D-a| + |2X-D-b| + Bn[D]
static void bwdStep(const Arr& Bn, int a, int b, Arr& Bk) {
    static int u[N], v[N], dq[N], p1[N];
    for (int D = 0; D < N; D++) {
        int e = Bn[D] >= INF ? INF : Bn[D] + abs(D - a);
        u[D] = e >= INF ? INF : e - D; v[D] = e >= INF ? INF : e + D;
    }
    int h = 0, t = 0, nx = 0;
    for (int X = 0; X < N; X++) {
        int c = 2 * X - b;
        int L = max(0, 2 * X - (N - 1)), R = min(N - 1, c);
        while (nx <= R) { while (t > h && u[dq[t - 1]] >= u[nx]) t--; dq[t++] = nx++; }
        while (t > h && dq[h] < L) h++;
        p1[X] = (t > h && L <= R && u[dq[h]] < INF) ? u[dq[h]] + c : INF;
    }
    h = 0; t = 0; nx = 0;
    for (int X = 0; X < N; X++) {
        int c = 2 * X - b;
        int L = max(0, c), R = min(N - 1, 2 * X);
        while (nx <= R) { while (t > h && v[dq[t - 1]] >= v[nx]) t--; dq[t++] = nx++; }
        while (t > h && dq[h] < L) h++;
        int p2 = (t > h && L <= R && v[dq[h]] < INF) ? v[dq[h]] - c : INF;
        Bk[X] = min(p1[X], p2);
    }
}
static inline int minSum(const Arr& a, const Arr& b) {
    int m = INF; for (int x = 0; x < N; x++) { int s = a[x] + b[x]; if (s < m) m = s; } return m;
}
static inline int argMinSum(const Arr& a, const Arr& b) {
    int m = INF, bi = 0; for (int x = 0; x < N; x++) { int s = a[x] + b[x]; if (s < m) { m = s; bi = x; } } return bi;
}
static inline int minArr(const Arr& a) { int m = INF; for (int x = 0; x < N; x++) m = min(m, a[x]); return m; }

// 슬롯 하나의 사슬: 이 슬롯을 쓰는 하위 구간 id 들(오름차순 = 시간순)과 축별 DP 배열
struct Chain {
    vector<int> legs;
    vector<Arr> F[2];  // F[k]: 앞 k 개 처리 후 새 원 위치별 최소 비용 (k = 0..n)
    vector<Arr> R[2];  // R[k] = B_{n+1-k}: 마지막 k 개의 비용 (그 첫 하위 구간의 거울 원 위치별), R[0] = 0
    int cost = 0;      // 텔레포트 수 포함
    const Arr& Bk(int ax, int k) const { return R[ax][legs.size() + 1 - k]; }  // 1-base k번째의 후방 배열
    void finishCost() { int nn = legs.size(); cost = nn; for (int ax = 0; ax < 2; ax++) cost += minArr(F[ax][nn]); }
    // F 는 p 이후, R 은 keepR 이후만 다시 계산 (나머지는 그대로 유효)
    void rebuild(int p, int keepR) {
        int nn = legs.size();
        for (int ax = 0; ax < 2; ax++) {
            F[ax].resize(nn + 1); R[ax].resize(nn + 1);
            F[ax][0] = ZERO; R[ax][0] = ZERO;
            for (int k = p + 1; k <= nn; k++) fwdStep(F[ax][k - 1], SA_[ax][legs[k - 1]], SB_[ax][legs[k - 1]], F[ax][k]);
            for (int k = keepR + 1; k <= nn; k++) { int id = legs[nn - k]; bwdStep(R[ax][k - 1], SA_[ax][id], SB_[ax][id], R[ax][k]); }
        }
        finishCost();
    }
    // [p, p+oc) 위치의 하위 구간들을 ids 로 교체
    void replaceBlock(int p, int oc, const int* ids, int cnt) {
        int keepR = (int)legs.size() - p - oc;
        legs.erase(legs.begin() + p, legs.begin() + p + oc);
        legs.insert(legs.begin() + p, ids, ids + cnt);
        rebuild(p, keepR);
    }
    int pos(int id) const { return lower_bound(legs.begin(), legs.end(), id) - legs.begin(); }
};

// 구간 설정: 나뉘지 않으면 c1 (-1 = 걷기만), 나뉘면 경유점 (mx,my) 와 두 하위 구간의 슬롯 c1, c2
struct LegCfg { bool split = false; int c1 = -1, c2 = -1; int mx = 0, my = 0; };
struct Sub { int id, slot, a[2], b[2]; };

int main() {
    auto t0 = chrono::steady_clock::now();
    auto elapsed = [&]() { return chrono::duration<double>(chrono::steady_clock::now() - t0).count(); };

    int n, K, TT;
    if (scanf("%d %d %d %d", &n, &M, &K, &TT) != 4) return 0;
    if (scanf("%d %d", &Gx[0], &Gy[0]) != 2) return 0;
    for (int s = 0; s < M; s++) if (scanf("%d %d", &Gx[s + 1], &Gy[s + 1]) != 2) return 0;
    ZERO.fill(0);

    vector<LegCfg> cfg(M);
    auto setParams = [&](int s, const LegCfg& L) {
        SA_[0][2 * s] = Gx[s]; SA_[1][2 * s] = Gy[s];
        if (!L.split) { SB_[0][2 * s] = Gx[s + 1]; SB_[1][2 * s] = Gy[s + 1]; return; }
        SB_[0][2 * s] = L.mx; SB_[1][2 * s] = L.my;
        SA_[0][2 * s + 1] = L.mx; SA_[1][2 * s + 1] = L.my;
        SB_[0][2 * s + 1] = Gx[s + 1]; SB_[1][2 * s + 1] = Gy[s + 1];
    };
    auto legDist = [&](int s) { return abs(Gx[s + 1] - Gx[s]) + abs(Gy[s + 1] - Gy[s]); };
    auto subsOf = [&](int s, const LegCfg& L, Sub* out) {
        if (!L.split) { out[0] = {2 * s, L.c1, {Gx[s], Gy[s]}, {Gx[s + 1], Gy[s + 1]}}; return 1; }
        out[0] = {2 * s, L.c1, {Gx[s], Gy[s]}, {L.mx, L.my}};
        out[1] = {2 * s + 1, L.c2, {L.mx, L.my}, {Gx[s + 1], Gy[s + 1]}};
        return 2;
    };

    // 초기: 모든 구간을 텔레포트 1회로, 슬롯은 순환 배정
    Chain ch[3];
    for (int s = 0; s < M; s++) { cfg[s].c1 = s % 3; setParams(s, cfg[s]); ch[s % 3].legs.push_back(2 * s); }
    for (int j = 0; j < 3; j++) ch[j].rebuild(0, 0);
    int cur = ch[0].cost + ch[1].cost + ch[2].cost, best = cur;
    vector<LegCfg> bestCfg = cfg;

    // 구간 s 의 설정을 nw 로 바꿀 때의 비용 변화
    Arr g1, g2;
    auto evalMove = [&](int s, const LegCfg& nw) {
        Sub os[2], ns[2]; int on = subsOf(s, cfg[s], os), nn = subsOf(s, nw, ns);
        int delta = 0;
        for (int i = 0; i < on; i++) if (os[i].slot < 0) delta -= legDist(s);
        for (int i = 0; i < nn; i++) if (ns[i].slot < 0) delta += legDist(s);
        for (int J = 0; J < 3; J++) {
            int oc = 0, nc = 0;
            for (int i = 0; i < on; i++) if (os[i].slot == J) oc++;
            for (int i = 0; i < nn; i++) if (ns[i].slot == J) nc++;
            if (oc == 0 && nc == 0) continue;
            Chain& C = ch[J]; int p = C.pos(2 * s);
            int v = (int)C.legs.size() - oc + nc;
            for (int ax = 0; ax < 2; ax++) {
                const Arr* f = &C.F[ax][p];
                Arr* bufs[2] = {&g1, &g2}; int bi = 0;
                for (int i = 0; i < nn; i++) if (ns[i].slot == J) { fwdStep(*f, ns[i].a[ax], ns[i].b[ax], *bufs[bi]); f = bufs[bi]; bi ^= 1; }
                v += minSum(*f, C.Bk(ax, p + oc + 1));
            }
            delta += v - C.cost;
        }
        return delta;
    };
    auto applyMove = [&](int s, const LegCfg& nw) {
        Sub os[2], ns[2]; int on = subsOf(s, cfg[s], os), nn = subsOf(s, nw, ns);
        int pJ[3], ocJ[3] = {0, 0, 0};
        for (int J = 0; J < 3; J++) pJ[J] = ch[J].pos(2 * s);
        for (int i = 0; i < on; i++) if (os[i].slot >= 0) ocJ[os[i].slot]++;
        cfg[s] = nw; setParams(s, nw);
        for (int J = 0; J < 3; J++) {
            int ids[2], c = 0;
            for (int i = 0; i < nn; i++) if (ns[i].slot == J) ids[c++] = ns[i].id;
            if (ocJ[J] == 0 && c == 0) continue;
            ch[J].replaceBlock(pJ[J], ocJ[J], ids, c);
        }
    };
    // 구간 s 직전(그 구간 자신의 하위 구간 제외)에 슬롯 J 의 원이 놓일 최적 위치
    auto circAt = [&](int J, int s, int& x, int& y) {
        Chain& C = ch[J]; int p = C.pos(2 * s);
        int oc = 0; while (p + oc < (int)C.legs.size() && C.legs[p + oc] <= 2 * s + 1) oc++;
        x = argMinSum(C.F[0][p], C.Bk(0, p + oc + 1)); y = argMinSum(C.F[1][p], C.Bk(1, p + oc + 1));
    };
    // 경유점 후보: 첫 텔레포트 착지점 (a 에서 출발) 또는 둘째 텔레포트가 b 에 정확히 착지하는 출발점
    auto guessWaypoint = [&](int s, LegCfg& L, mt19937& rng) {
        int x1, y1, x2, y2; circAt(L.c1, s, x1, y1); circAt(L.c2, s, x2, y2);
        if (L.c1 != L.c2 && (rng() & 1)) { L.mx = 2 * x2 - Gx[s + 1]; L.my = 2 * y2 - Gy[s + 1]; }
        else { L.mx = 2 * x1 - Gx[s]; L.my = 2 * y1 - Gy[s]; }
        L.mx = min(max(L.mx, 0), N - 1); L.my = min(max(L.my, 0), N - 1);
    };

    mt19937 rng(12345);
    double temp = TEMP0;
    auto accept = [&](int delta) { return delta <= 0 || exp(-delta / temp) * 4294967296.0 > rng(); };
    for (long long it = 0;; it++) {
        if ((it & 63) == 0) {
            double el = elapsed();
            if (el > TIME_LIMIT) break;
            temp = TEMP0 * pow(TEMP1 / TEMP0, el / TIME_LIMIT);
        }
        if (rng() % 100 < 90) {
            // 구간 하나의 설정 변경
            int s = rng() % M;
            LegCfg nw = cfg[s];
            if (!nw.split) {
                if ((int)(rng() % 100) < SPLIT_TRY) {
                    nw.split = true; nw.c1 = rng() % 3; nw.c2 = rng() % 3;
                    guessWaypoint(s, nw, rng);
                } else {
                    int c = (int)(rng() % 4) - 1;
                    if (c == nw.c1) continue;
                    nw.c1 = c;
                }
            } else {
                int q = rng() % 100;
                if (q < 45) {
                    int d = 1 + rng() % 6;
                    nw.mx = min(max(nw.mx + (int)(rng() % (2 * d + 1)) - d, 0), N - 1);
                    nw.my = min(max(nw.my + (int)(rng() % (2 * d + 1)) - d, 0), N - 1);
                } else if (q < 65) {
                    if (rng() & 1) nw.c1 = rng() % 3; else nw.c2 = rng() % 3;
                } else if (q < 85) {
                    nw.split = false; nw.c1 = (int)(rng() % 4) - 1;
                } else guessWaypoint(s, nw, rng);
            }
            int delta = evalMove(s, nw);
            if (accept(delta)) {
                applyMove(s, nw); cur += delta;
                if (cur < best) { best = cur; bestCfg = cfg; }
            }
        } else {
            // 두 슬롯의 사슬을 시각 tt 이후로 맞바꿈
            int j1 = rng() % 3, j2 = rng() % 3;
            if (j1 == j2) continue;
            int tt = rng() % (2 * M + 1);
            Chain& A = ch[j1]; Chain& B = ch[j2];
            int p1 = A.pos(tt), p2 = B.pos(tt);
            int n1 = A.legs.size(), n2 = B.legs.size();
            if (p1 == n1 && p2 == n2) continue;
            int vA = p1 + (n2 - p2), vB = p2 + (n1 - p1);
            for (int ax = 0; ax < 2; ax++) { vA += minSum(A.F[ax][p1], B.Bk(ax, p2 + 1)); vB += minSum(B.F[ax][p2], A.Bk(ax, p1 + 1)); }
            int delta = vA + vB - A.cost - B.cost;
            if (accept(delta)) {
                int tA = n1 - p1, tB = n2 - p2;
                vector<int> la(A.legs.begin(), A.legs.begin() + p1), lb(B.legs.begin(), B.legs.begin() + p2);
                la.insert(la.end(), B.legs.begin() + p2, B.legs.end());
                lb.insert(lb.end(), A.legs.begin() + p1, A.legs.end());
                for (int ax = 0; ax < 2; ax++) {  // 꼬리의 후방 배열은 그대로 옮긴다
                    vector<Arr> rA(A.R[ax].begin(), A.R[ax].begin() + tA + 1), rB(B.R[ax].begin(), B.R[ax].begin() + tB + 1);
                    A.R[ax] = rB; B.R[ax] = rA;
                }
                A.legs = la; B.legs = lb;
                for (int id : A.legs) { int s = id / 2; if (id & 1) cfg[s].c2 = j1; else cfg[s].c1 = j1; }
                for (int id : B.legs) { int s = id / 2; if (id & 1) cfg[s].c2 = j2; else cfg[s].c1 = j2; }
                A.rebuild(p1, tB); B.rebuild(p2, tA);
                cur += delta;
                if (cur < best) { best = cur; bestCfg = cfg; }
            }
        }
    }

    // 복원: 최선 설정으로 사슬을 다시 계산하고 축별로 역추적해 출발점/초기 원 위치를 정한다
    cfg = bestCfg;
    for (int j = 0; j < 3; j++) ch[j].legs.clear();
    for (int s = 0; s < M; s++) {
        setParams(s, cfg[s]);
        if (cfg[s].c1 >= 0) ch[cfg[s].c1].legs.push_back(2 * s);
        if (cfg[s].split) ch[cfg[s].c2].legs.push_back(2 * s + 1);
    }
    vector<array<int, 2>> dep(2 * M);
    int init[3][2];
    for (int j = 0; j < 3; j++) {
        Chain& C = ch[j];
        sort(C.legs.begin(), C.legs.end());
        C.rebuild(0, 0);
        int nn = C.legs.size();
        for (int ax = 0; ax < 2; ax++) {
            if (nn == 0) { init[j][ax] = N / 2; continue; }
            int D = 0;
            for (int x = 1; x < N; x++) if (C.F[ax][nn][x] < C.F[ax][nn][D]) D = x;
            for (int k = nn; k >= 1; k--) {
                int id = C.legs[k - 1], b = SB_[ax][id];
                int bx = -1, bv = INF;
                for (int X = 0; X < N; X++) {
                    int l = 2 * X - D;
                    if (l < 0 || l > N - 1 || C.F[ax][k - 1][X] >= INF) continue;
                    int v = C.F[ax][k - 1][X] + abs(l - b);
                    if (v < bv) { bv = v; bx = X; }
                }
                dep[id][ax] = D; D = bx;
            }
            init[j][ax] = D;
        }
    }

    string out;
    for (int j = 0; j < 3; j++) out += to_string(init[j][0]) + " " + to_string(init[j][1]) + "\n";
    int px = Gx[0], py = Gy[0], cx[3], cy[3];
    for (int j = 0; j < 3; j++) { cx[j] = init[j][0]; cy[j] = init[j][1]; }
    long long A = 0;
    auto walk = [&](int tx, int ty) {
        while (px < tx) { out += "D\n"; px++; A++; }
        while (px > tx) { out += "U\n"; px--; A++; }
        while (py < ty) { out += "R\n"; py++; A++; }
        while (py > ty) { out += "L\n"; py--; A++; }
    };
    auto jump = [&](int j) {
        out += "J " + to_string(cx[j]) + " " + to_string(cy[j]) + "\n"; A++;
        int lx = 2 * cx[j] - px, ly = 2 * cy[j] - py;
        cx[j] = px; cy[j] = py; px = lx; py = ly;
    };
    for (int s = 0; s < M; s++) {
        const LegCfg& L = cfg[s];
        if (L.c1 >= 0) { walk(dep[2 * s][0], dep[2 * s][1]); jump(L.c1); }
        if (L.split) { walk(L.mx, L.my); walk(dep[2 * s + 1][0], dep[2 * s + 1][1]); jump(L.c2); }
        walk(Gx[s + 1], Gy[s + 1]);
    }
    fwrite(out.data(), 1, out.size(), stdout);
    fprintf(stderr, "A=%lld model=%d time=%.3f\n", A, best, elapsed());
}
