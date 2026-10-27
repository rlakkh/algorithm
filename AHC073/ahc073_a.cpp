// AHC073 A - Circles of Inversion
//
// 모델: 각 구간(직전 목적지 a -> 다음 목적지 b)은 "걷기만" 하거나 "텔레포트 1회"로 처리한다.
//   텔레포트 구간: a 에서 출발점 D 까지 걷고, 원 X 를 써서 2X - D 로 착지한 뒤 b 까지 걷는다.
//   비용 = |D - a| + 1 + |2X - D - b| (L1), 쓴 원 X 는 사라지고 D 에 새 원이 생긴다.
//
// 핵심 관찰
//  1) 원 3개를 슬롯으로 보면, 텔레포트 구간 s 가 슬롯 j 를 쓰면 그 슬롯의 원은 "슬롯 j 를 직전에 쓴
//     구간의 출발점"(없으면 초기 원) 이고, 구간 s 의 출발점이 슬롯 j 의 새 원이 된다.
//     -> 각 구간이 어느 슬롯을 쓰는지(배정)만 정하면, 슬롯마다 원 위치들이 사슬로 이어지고
//        서로 다른 슬롯은 완전히 독립이다. 게다가 x, y 축도 독립이다.
//  2) 따라서 배정이 주어지면 비용 최소의 출발점/원 위치는 슬롯별·축별 사슬 DP 로 정확히 구해진다.
//        F_k(D) = |D - a_k| + min_{X: 0 <= 2X-D <= 127} F_{k-1}(X) + |2X - D - b_k|,  F_0 = 0
//     (초기 원 위치도 자유롭게 고를 수 있으므로 F_0 = 0). 내부 min 은 X 를 2X <= D+b, 2X >= D+b 로
//     나누면 구간 최솟값 두 개가 되고, 구간 양 끝이 D 에 대해 단조라 단조 덱으로 O(128) 에 계산된다.
//  3) 전방 F 와 후방 B 배열을 들고 있으면 "구간 하나의 슬롯 바꾸기", "두 슬롯의 꼬리 교환" 같은
//     변경의 비용 변화를 O(128) 에 평가할 수 있다 -> 배정에 대한 담금질(SA).
//  4) 원이 가장자리 근처에서 죽는 문제 등은 모두 DP 비용에 정확히 반영되므로 별도 휴리스틱이 필요 없다.
//     (목적지를 일찍 지나가도 방문 순서는 깨지지 않으므로 경로 검사는 필요 없다.)
#include <bits/stdc++.h>
using namespace std;

static const int N = 128, INF = 1000000000, MAXM = 160;
typedef array<int, N> Arr;

const double TIME_LIMIT = 1.75;      // 담금질 시간
const double TEMP0 = 20, TEMP1 = 0.5;

int M;
int LA[2][MAXM], LB[2][MAXM], distL[MAXM];  // 구간 시작/끝 좌표 (축별), 걷기만 할 때의 비용
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
static inline int minArr(const Arr& a) { int m = INF; for (int x = 0; x < N; x++) m = min(m, a[x]); return m; }

// 슬롯 하나의 사슬: 이 슬롯을 쓰는 구간들(오름차순)과 축별 전방/후방 DP 배열
struct Chain {
    vector<int> legs;
    vector<Arr> F[2], B[2];  // F[k]: 앞 k 구간 처리 후 새 원 위치별 최소 비용 (k=0..n)
                             // B[k]: k번째 구간(1-base)의 거울 원 위치별 이후 최소 비용 (k=1..n+1)
    int cost = 0;            // 텔레포트 수 포함
    void recompute() {
        int n = legs.size();
        for (int ax = 0; ax < 2; ax++) {
            F[ax].resize(n + 1); B[ax].resize(n + 2);
            F[ax][0] = ZERO;
            for (int k = 1; k <= n; k++) fwdStep(F[ax][k - 1], LA[ax][legs[k - 1]], LB[ax][legs[k - 1]], F[ax][k]);
            B[ax][n + 1] = ZERO;
            for (int k = n; k >= 1; k--) bwdStep(B[ax][k + 1], LA[ax][legs[k - 1]], LB[ax][legs[k - 1]], B[ax][k]);
        }
        cost = n;
        for (int ax = 0; ax < 2; ax++) cost += minArr(F[ax][n]);
    }
    int pos(int s) const { return lower_bound(legs.begin(), legs.end(), s) - legs.begin(); }
};

int main() {
    auto t0 = chrono::steady_clock::now();
    auto elapsed = [&]() { return chrono::duration<double>(chrono::steady_clock::now() - t0).count(); };

    int n, K, TT;
    if (scanf("%d %d %d %d", &n, &M, &K, &TT) != 4) return 0;
    int Sx, Sy;
    if (scanf("%d %d", &Sx, &Sy) != 2) return 0;
    vector<pair<int, int>> Gs(M);
    for (auto& p : Gs) if (scanf("%d %d", &p.first, &p.second) != 2) return 0;
    for (int s = 0; s < M; s++) {
        LA[0][s] = s == 0 ? Sx : Gs[s - 1].first; LA[1][s] = s == 0 ? Sy : Gs[s - 1].second;
        LB[0][s] = Gs[s].first; LB[1][s] = Gs[s].second;
        distL[s] = abs(LB[0][s] - LA[0][s]) + abs(LB[1][s] - LA[1][s]);
    }
    ZERO.fill(0);

    // 배정 c[s]: -1 = 걷기만, 0..2 = 사용할 원 슬롯. 초기값은 순환 배정.
    vector<int> c(M);
    Chain ch[3];
    for (int s = 0; s < M; s++) { c[s] = s % 3; ch[s % 3].legs.push_back(s); }
    for (int j = 0; j < 3; j++) ch[j].recompute();
    int cur = ch[0].cost + ch[1].cost + ch[2].cost, best = cur;
    vector<int> bestC = c;

    mt19937 rng(12345);
    double temp = TEMP0;
    Arr g;
    auto accept = [&](int delta) { return delta <= 0 || exp(-delta / temp) * 4294967296.0 > rng(); };
    for (long long it = 0;; it++) {
        if ((it & 63) == 0) {
            double el = elapsed();
            if (el > TIME_LIMIT) break;
            temp = TEMP0 * pow(TEMP1 / TEMP0, el / TIME_LIMIT);
        }
        if (rng() % 10 < 8) {
            // 구간 s 의 배정 변경 (슬롯 <-> 슬롯 / 걷기)
            int s = rng() % M, old = c[s], nw = (int)(rng() % 4) - 1;
            if (nw == old) continue;
            int delta = 0;
            if (old >= 0) {
                Chain& C = ch[old]; int k = C.pos(s) + 1;
                int v = (int)C.legs.size() - 1;
                for (int ax = 0; ax < 2; ax++) v += minSum(C.F[ax][k - 1], C.B[ax][k + 1]);
                delta += v - C.cost;
            } else delta -= distL[s];
            if (nw >= 0) {
                Chain& C = ch[nw]; int p = C.pos(s);
                int v = (int)C.legs.size() + 1;
                for (int ax = 0; ax < 2; ax++) { fwdStep(C.F[ax][p], LA[ax][s], LB[ax][s], g); v += minSum(g, C.B[ax][p + 1]); }
                delta += v - C.cost;
            } else delta += distL[s];
            if (accept(delta)) {
                if (old >= 0) { auto& L = ch[old].legs; L.erase(L.begin() + ch[old].pos(s)); ch[old].recompute(); }
                if (nw >= 0) { auto& L = ch[nw].legs; L.insert(L.begin() + ch[nw].pos(s), s); ch[nw].recompute(); }
                c[s] = nw; cur += delta;
                if (cur < best) { best = cur; bestC = c; }
            }
        } else {
            // 두 슬롯의 사슬을 시각 tt 이후로 맞바꿈
            int j1 = rng() % 3, j2 = rng() % 3;
            if (j1 == j2) continue;
            int tt = rng() % (M + 1);
            Chain& A = ch[j1]; Chain& B = ch[j2];
            int p1 = A.pos(tt), p2 = B.pos(tt);
            int n1 = A.legs.size(), n2 = B.legs.size();
            if (p1 == n1 && p2 == n2) continue;
            int vA = p1 + (n2 - p2), vB = p2 + (n1 - p1);
            for (int ax = 0; ax < 2; ax++) { vA += minSum(A.F[ax][p1], B.B[ax][p2 + 1]); vB += minSum(B.F[ax][p2], A.B[ax][p1 + 1]); }
            int delta = vA + vB - A.cost - B.cost;
            if (accept(delta)) {
                vector<int> la(A.legs.begin(), A.legs.begin() + p1), lb(B.legs.begin(), B.legs.begin() + p2);
                la.insert(la.end(), B.legs.begin() + p2, B.legs.end());
                lb.insert(lb.end(), A.legs.begin() + p1, A.legs.end());
                A.legs = la; B.legs = lb;
                for (int s : A.legs) c[s] = j1;
                for (int s : B.legs) c[s] = j2;
                A.recompute(); B.recompute();
                cur += delta;
                if (cur < best) { best = cur; bestC = c; }
            }
        }
    }

    // 복원: 최선 배정으로 사슬을 다시 계산하고 축별로 역추적해 출발점/거울 위치를 정한다
    for (int j = 0; j < 3; j++) ch[j].legs.clear();
    for (int s = 0; s < M; s++) if (bestC[s] >= 0) ch[bestC[s]].legs.push_back(s);
    vector<array<int, 2>> dep(M);
    int init[3][2];
    for (int j = 0; j < 3; j++) {
        Chain& C = ch[j]; C.recompute();
        int nn = C.legs.size();
        for (int ax = 0; ax < 2; ax++) {
            if (nn == 0) { init[j][ax] = N / 2; continue; }
            int D = 0;
            for (int x = 1; x < N; x++) if (C.F[ax][nn][x] < C.F[ax][nn][D]) D = x;
            for (int k = nn; k >= 1; k--) {
                int s = C.legs[k - 1], b = LB[ax][s];
                int bx = -1, bv = INF;
                for (int X = 0; X < N; X++) {
                    int l = 2 * X - D;
                    if (l < 0 || l > N - 1 || C.F[ax][k - 1][X] >= INF) continue;
                    int v = C.F[ax][k - 1][X] + abs(l - b);
                    if (v < bv) { bv = v; bx = X; }
                }
                dep[s][ax] = D; D = bx;
            }
            init[j][ax] = D;
        }
    }

    string out;
    for (int j = 0; j < 3; j++) out += to_string(init[j][0]) + " " + to_string(init[j][1]) + "\n";
    int px = Sx, py = Sy, cx[3], cy[3];
    for (int j = 0; j < 3; j++) { cx[j] = init[j][0]; cy[j] = init[j][1]; }
    long long A = 0;
    auto walk = [&](int tx, int ty) {
        while (px < tx) { out += "D\n"; px++; A++; }
        while (px > tx) { out += "U\n"; px--; A++; }
        while (py < ty) { out += "R\n"; py++; A++; }
        while (py > ty) { out += "L\n"; py--; A++; }
    };
    for (int s = 0; s < M; s++) {
        int j = bestC[s];
        if (j >= 0) {
            walk(dep[s][0], dep[s][1]);
            out += "J " + to_string(cx[j]) + " " + to_string(cy[j]) + "\n"; A++;
            int lx = 2 * cx[j] - px, ly = 2 * cy[j] - py;
            cx[j] = px; cy[j] = py; px = lx; py = ly;
        }
        walk(Gs[s].first, Gs[s].second);
    }
    fwrite(out.data(), 1, out.size(), stdout);
    fprintf(stderr, "A=%lld model=%d time=%.3f\n", A, best, elapsed());
}
