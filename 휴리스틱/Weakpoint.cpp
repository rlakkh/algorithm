#include <bits/stdc++.h>
using namespace std;

/*
 * Weakpoint (N = 200, 시간 제한 2초)
 *
 * 상자 b 를 무기 w 로 치려면 w 가 b 보다 먼저 열려 있어야 하므로, 해는
 * "무기 w 의 사용 하나 -> 상자 b" 간선들로 이루어진 DAG 와 그 위상 순서로 표현된다.
 * 공격 횟수 T = Σ_b ( b 에 들어간 무기 타격 수 + max(0, H_b - 받은 피해) ).
 * 무기 타격이 없는 상자는 맨손으로만 연다 (첫 상자 = 루트).
 *
 * 무기 사용의 배정을 담금질로 최적화한다.
 *  - 이동: 무기 중심 재배정, 상자 중심 재배정, 두 사용의 대상 교환
 *  - 사이클이 생기는 재배정은 b -> ... -> w 경로에서 손해가 가장 작은 간선을 빼고(최대 2개)
 *    밀려난 사용을 다른 상자로 다시 보낸다
 *  - 초반(구조가 정해지는 구간)은 체인 3개로 돌려 가장 좋은 것만 이어서 식힌다
 * 위상 순서는 간선을 추가할 때만 국소적으로 고친다.
 */

static const int N = 200;
static const int MAXC = 6;
static const int MW = 20;    // 무기마다 강한 상자 후보 수
static const int MB = 10;    // 상자마다 강한 무기 후보 수
static const int EJMAX = 2;  // 사이클을 끊기 위해 뺄 수 있는 간선 수

static const double TIME_LIMIT = 1.85;
static const double T0 = 12.0, T1 = 0.5;
static const int PSWAP = 30, PBOX = 30;   // 교환 30%, 나머지 중 상자 중심 30%
static const int CHAINS = 3;
static const double SPLIT = 0.45;          // 체인들이 진행하는 스케줄 구간 [0, SPLIT]
static const double SPLIT_TIME = 0.5;      // 그 구간에 쓰는 시간 비율

int H[N], C[N], A[N][N];
int tgt[N][MAXC];      // 무기 w 의 u 번째 사용이 치는 상자 (-1: 미사용)
int R[N], K[N];        // R = H - (받은 피해 합), K = 받은 무기 타격 수
int ordp[N], byOrd[N]; // 현재 그래프의 위상 순서
int cand[N][MW], att[N][MB];
int hit[N][N * MAXC], hitn[N]; // 상자별 (무기*8+사용) 목록
int useCode[N * MAXC], U;

struct RNG {
	uint64_t x = 88172645463325252ULL;
	inline uint64_t next() { x ^= x << 7; x ^= x >> 9; return x; }
	inline int below(uint32_t bits16, int n) { return (int)((bits16 * (uint32_t)n) >> 16); }
} rng;

static float negLog[1 << 16];

inline int cst(int r) { return r > 0 ? r : 0; }
// 상자 b 에서 피해 a 짜리 타격 하나를 빼거나 더할 때 비용 변화
inline int dRemove(int b, int a) { return -1 + cst(R[b] + a) - cst(R[b]); }
inline int dAdd(int b, int a) { return 1 + cst(R[b] - a) - cst(R[b]); }

int visStamp[N], par[N], stampCur = 0;
int pathBuf[N + 1], pathLen;

// 간선 w->b 를 추가할 수 있으면 위상 순서를 고치고 true.
// 사이클이 생기면 false 와 함께 b -> ... -> w 경로를 pathBuf 에 남긴다.
bool tryAddEdge(int w, int b) {
	if (ordp[w] < ordp[b]) return true;
	int lo = ordp[b], hi = ordp[w];
	++stampCur;
	static int stk[N];
	int sp = 0;
	stk[sp++] = b; visStamp[b] = stampCur; par[b] = -1;
	while (sp) {
		int v = stk[--sp];
		for (int u = 0; u < C[v]; u++) {
			int x = tgt[v][u];
			if (x < 0) continue;
			if (x == w) {
				pathLen = 0;
				pathBuf[pathLen++] = w;
				for (int y = v; y != -1; y = par[y]) pathBuf[pathLen++] = y;
				reverse(pathBuf, pathBuf + pathLen);
				return false;
			}
			if (ordp[x] > hi || visStamp[x] == stampCur) continue;
			visStamp[x] = stampCur; par[x] = v;
			stk[sp++] = x;
		}
	}
	// 구간 [lo, hi] 안에서 b 에서 닿는 노드들을 w 뒤로 보낸다
	static int tmp[N];
	int t = 0;
	for (int p = lo; p <= hi; p++) if (visStamp[byOrd[p]] != stampCur) tmp[t++] = byOrd[p];
	for (int p = lo; p <= hi; p++) if (visStamp[byOrd[p]] == stampCur) tmp[t++] = byOrd[p];
	for (int i = 0; i < t; i++) { byOrd[lo + i] = tmp[i]; ordp[tmp[i]] = lo + i; }
	return true;
}

bool fullTopo() {
	static int indeg[N], q[N];
	fill(indeg, indeg + N, 0);
	for (int w = 0; w < N; w++) for (int u = 0; u < C[w]; u++) if (tgt[w][u] >= 0) indeg[tgt[w][u]]++;
	int qh = 0, qt = 0;
	for (int v = 0; v < N; v++) if (!indeg[v]) q[qt++] = v;
	while (qh < qt) {
		int v = q[qh++];
		for (int u = 0; u < C[v]; u++) {
			int x = tgt[v][u];
			if (x >= 0 && --indeg[x] == 0) q[qt++] = x;
		}
	}
	if (qt < N) return false;
	for (int i = 0; i < N; i++) { byOrd[i] = q[i]; ordp[q[i]] = i; }
	return true;
}

inline void setUse(int w, int u, int nb) {
	int ob = tgt[w][u], code = w * 8 + u;
	if (ob >= 0) {
		R[ob] += A[w][ob]; K[ob]--;
		int* h = hit[ob];
		for (int i = 0; i < hitn[ob]; i++) if (h[i] == code) { h[i] = h[--hitn[ob]]; break; }
	}
	if (nb >= 0) { R[nb] -= A[w][nb]; K[nb]++; hit[nb][hitn[nb]++] = code; }
	tgt[w][u] = nb;
}

int cur, best;
int bestTgt[N][MAXC];
bool trackBest = false;

// 무기 w 의 사용 u 를 nb 로 옮기는 이동. 사이클이 생기면 경로의 간선을 빼서 시도한다.
inline void relocate(int w, int u, int nb, float thr) {
	int ob = tgt[w][u];
	if (nb == ob) return;
	int delta = 0;
	if (ob >= 0) delta += dRemove(ob, A[w][ob]);
	if (nb >= 0) delta += dAdd(nb, A[w][nb]);
	if (delta > thr) return;
	if (nb < 0 || tryAddEdge(w, nb)) {
		setUse(w, u, nb);
		cur += delta;
		return;
	}

	static int sOrd[N], sBy[N];
	int remW[EJMAX * MAXC], remU[EJMAX * MAXC], remB[EJMAX * MAXC], nrem = 0;
	int total = delta;
	bool ok = false, saved = false;
	for (int e = 0; e < EJMAX; e++) {
		// 경로 nb -> ... -> w 위에서 빼도 손해가 가장 작은 간선 (x 가 y 를 친다)
		int bestLoss = INT_MAX, bx = -1, by = -1;
		for (int i = 0; i + 1 < pathLen; i++) {
			int x = pathBuf[i], y = pathBuf[i + 1];
			if (y == ob || y == nb || x == w) continue;
			int m = 0, dmg = 0;
			for (int uu = 0; uu < C[x]; uu++) if (tgt[x][uu] == y) { m++; dmg += A[x][y]; }
			int loss = -m + cst(R[y] + dmg) - cst(R[y]);
			if (loss < bestLoss) { bestLoss = loss; bx = x; by = y; }
		}
		if (bx < 0) break;
		// 밀려난 사용을 다른 후보로 보냈을 때의 이득 (사이클 무시한 낙관적 추정)
		int rcnt = 0, gain = 0;
		for (int uu = 0; uu < C[bx]; uu++) if (tgt[bx][uu] == by) rcnt++;
		for (int i = 0; i < MW; i++) {
			int t = cand[bx][i];
			if (t == by || t == nb || t == ob) continue;
			gain = min(gain, dAdd(t, A[bx][t]));
		}
		if (total + bestLoss + gain * rcnt > thr) break;
		if (!saved) { memcpy(sOrd, ordp, sizeof(ordp)); memcpy(sBy, byOrd, sizeof(byOrd)); saved = true; }
		for (int uu = 0; uu < C[bx]; uu++) if (tgt[bx][uu] == by) {
			remW[nrem] = bx; remU[nrem] = uu; remB[nrem] = by; nrem++;
			setUse(bx, uu, -1);
		}
		total += bestLoss;
		if (tryAddEdge(w, nb)) { ok = true; break; }
	}
	if (!ok) {
		for (int i = 0; i < nrem; i++) setUse(remW[i], remU[i], remB[i]);
		return;
	}
	setUse(w, u, nb);
	for (int i = 0; i < nrem; i++) {
		int x = remW[i], bt = -1, bd = 0;
		for (int j = 0; j < MW; j++) {
			int t = cand[x][j];
			if (t == remB[i]) continue;
			int d = dAdd(t, A[x][t]);
			if (d < bd && tryAddEdge(x, t)) { bd = d; bt = t; }
		}
		if (bt >= 0) { setUse(x, remU[i], bt); total += bd; }
	}
	if (total > thr) {
		for (int i = 0; i < nrem; i++) setUse(remW[i], remU[i], remB[i]);
		setUse(w, u, ob);
		memcpy(ordp, sOrd, sizeof(ordp)); memcpy(byOrd, sBy, sizeof(byOrd));
		return;
	}
	cur += total;
}

// 두 사용의 대상을 맞바꾼다: w1 이 b2 를, w2 가 b1 을 친다 (b1 = -1 이면 w2 가 쉰다)
inline void swapTargets(int w1, int u1, int w2, int u2, float thr) {
	int b1 = tgt[w1][u1], b2 = tgt[w2][u2];
	int dd2 = A[w1][b2] - A[w2][b2];
	int delta = cst(R[b2] - dd2) - cst(R[b2]);
	if (b1 >= 0) { int dd1 = A[w2][b1] - A[w1][b1]; delta += cst(R[b1] - dd1) - cst(R[b1]); }
	if (delta > thr) return;
	// 두 간선을 빼고 새 간선을 하나씩 넣어 본다 (실패해도 위상 순서는 원래 그래프에 대해 유효하다)
	tgt[w1][u1] = -1; tgt[w2][u2] = -1;
	if (!tryAddEdge(w1, b2)) { tgt[w1][u1] = b1; tgt[w2][u2] = b2; return; }
	tgt[w1][u1] = b2;
	if (b1 >= 0 && !tryAddEdge(w2, b1)) { tgt[w1][u1] = b1; tgt[w2][u2] = b2; return; }
	tgt[w1][u1] = b1; tgt[w2][u2] = b2;
	setUse(w1, u1, -1); setUse(w2, u2, -1);
	setUse(w1, u1, b2); setUse(w2, u2, b1);
	cur += delta;
}

chrono::steady_clock::time_point startTime;
double elapsed() { return chrono::duration<double>(chrono::steady_clock::now() - startTime).count(); }

// 온도 스케줄 구간 [fFrom, fTo] 를 시간 [tA, tB] 동안 진행한다
void anneal(double fFrom, double fTo, double tA, double tB) {
	const int thrSwap = PSWAP * 655, thrBox = thrSwap + (100 - PSWAP) * PBOX * 655 / 100;
	double temp = T0;
	for (long long it = 0;; it++) {
		if ((it & 1023) == 0) {
			double g = (elapsed() - tA) / (tB - tA);
			if (g >= 1.0) break;
			temp = T0 * pow(T1 / T0, fFrom + (fTo - fFrom) * g);
		}
		uint64_t r = rng.next();
		uint32_t typ = (uint32_t)(r & 0xFFFF);
		float thr = (float)temp * negLog[(r >> 48) & 0xFFFF];
		if (typ >= (uint32_t)thrBox) {
			// 무기 중심: 사용 하나를 그 무기가 강한 상자로 (10% 는 해제, 5% 는 아무 상자)
			int code = useCode[rng.below((r >> 16) & 0xFFFF, U)];
			int w = code >> 3, u = code & 7, nb;
			uint32_t r2 = (r >> 32) & 0xFFFF;
			if (r2 < 6554 && tgt[w][u] >= 0) nb = -1;
			else if (r2 < 9830) { nb = rng.below(r2 * 20 & 0xFFFF, N); if (nb == w) continue; }
			else nb = cand[w][rng.below(r2 * 7 & 0xFFFF, MW)];
			relocate(w, u, nb, thr);
		} else if (typ >= (uint32_t)thrSwap) {
			// 상자 중심: 상자 nb 에 강한 무기 w 의 사용 중 빼도 손해가 가장 작은 것을 nb 로
			int nb = rng.below((r >> 16) & 0xFFFF, N);
			int w = att[nb][rng.below((r >> 32) & 0xFFFF, MB)];
			int bestLoss = INT_MAX, u = 0;
			for (int uu = 0; uu < C[w]; uu++) {
				int o = tgt[w][uu];
				int loss = (o < 0) ? 0 : (o == nb ? INT_MAX / 2 : dRemove(o, A[w][o]));
				if (loss < bestLoss) { bestLoss = loss; u = uu; }
			}
			relocate(w, u, nb, thr);
		} else {
			int code1 = useCode[rng.below((r >> 16) & 0xFFFF, U)];
			int w1 = code1 >> 3, u1 = code1 & 7;
			int b1 = tgt[w1][u1];
			int b2 = cand[w1][rng.below((r >> 32) & 0xFFFF, MW)];
			if (b2 == b1 || hitn[b2] == 0) continue;
			int code2 = hit[b2][rng.below((uint32_t)(r >> 8) & 0xFFFF, hitn[b2])];
			int w2 = code2 >> 3, u2 = code2 & 7;
			if (w2 == w1 || b1 == w2) continue;
			swapTargets(w1, u1, w2, u2, thr);
		}
		if (trackBest && cur < best) {
			best = cur;
			memcpy(bestTgt, tgt, sizeof(tgt));
		}
	}
}

struct State {
	int tgt[N][MAXC], R[N], K[N], ordp[N], byOrd[N], hitn[N], hit[N][N * MAXC], cur;
};
State initState, bestChain;

void save(State& s) {
	memcpy(s.tgt, tgt, sizeof(tgt)); memcpy(s.R, R, sizeof(R)); memcpy(s.K, K, sizeof(K));
	memcpy(s.ordp, ordp, sizeof(ordp)); memcpy(s.byOrd, byOrd, sizeof(byOrd)); memcpy(s.hitn, hitn, sizeof(hitn));
	for (int i = 0; i < N; i++) memcpy(s.hit[i], hit[i], sizeof(int) * hitn[i]);
	s.cur = cur;
}

void load(const State& s) {
	memcpy(tgt, s.tgt, sizeof(tgt)); memcpy(R, s.R, sizeof(R)); memcpy(K, s.K, sizeof(K));
	memcpy(ordp, s.ordp, sizeof(ordp)); memcpy(byOrd, s.byOrd, sizeof(byOrd)); memcpy(hitn, s.hitn, sizeof(hitn));
	for (int i = 0; i < N; i++) memcpy(hit[i], s.hit[i], sizeof(int) * hitn[i]);
	cur = s.cur;
}

int main() {
	startTime = chrono::steady_clock::now();

	int n;
	if (scanf("%d", &n) != 1) return 0;
	for (int i = 0; i < N; i++) if (scanf("%d", &H[i]) != 1) return 0;
	for (int i = 0; i < N; i++) if (scanf("%d", &C[i]) != 1) return 0;
	for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) if (scanf("%d", &A[i][j]) != 1) return 0;

	for (int i = 0; i < (1 << 16); i++) negLog[i] = (float)-log((i + 0.5) / (1 << 16));
	for (int w = 0; w < N; w++) {
		vector<int> v;
		for (int b = 0; b < N; b++) if (b != w) v.push_back(b);
		partial_sort(v.begin(), v.begin() + MW, v.end(), [&](int a, int b) { return A[w][a] > A[w][b]; });
		for (int i = 0; i < MW; i++) cand[w][i] = v[i];
	}
	for (int b = 0; b < N; b++) {
		vector<int> v;
		for (int w = 0; w < N; w++) if (w != b) v.push_back(w);
		partial_sort(v.begin(), v.begin() + MB, v.end(), [&](int x, int y) { return A[x][b] > A[y][b]; });
		for (int i = 0; i < MB; i++) att[b][i] = v[i];
	}
	for (int w = 0; w < N; w++) for (int u = 0; u < C[w]; u++) useCode[U++] = w * 8 + u;

	// 시작: 모든 상자를 맨손으로 연다
	for (int w = 0; w < N; w++) for (int u = 0; u < MAXC; u++) tgt[w][u] = -1;
	for (int i = 0; i < N; i++) { ordp[i] = i; byOrd[i] = i; R[i] = H[i]; K[i] = 0; hitn[i] = 0; }
	cur = 0;
	for (int b = 0; b < N; b++) cur += H[b];

	// 초반 구간을 체인 여러 개로 돌려 가장 좋은 상태만 이어서 식힌다
	double tStart = elapsed();
	double tSplit = tStart + (TIME_LIMIT - tStart) * SPLIT_TIME;
	save(initState);
	int bestChainCost = INT_MAX;
	for (int k = 0; k < CHAINS; k++) {
		load(initState);
		anneal(0.0, SPLIT, tStart + (tSplit - tStart) * k / CHAINS, tStart + (tSplit - tStart) * (k + 1) / CHAINS);
		if (cur < bestChainCost) { bestChainCost = cur; save(bestChain); }
	}
	load(bestChain);
	best = cur;
	memcpy(bestTgt, tgt, sizeof(tgt));
	trackBest = true;
	anneal(SPLIT, 1.0, tSplit, TIME_LIMIT);

	memcpy(tgt, bestTgt, sizeof(tgt));
	fullTopo();
	for (int b = 0; b < N; b++) { R[b] = H[b]; K[b] = 0; }
	for (int w = 0; w < N; w++) for (int u = 0; u < C[w]; u++) if (tgt[w][u] >= 0) { R[tgt[w][u]] -= A[w][tgt[w][u]]; K[tgt[w][u]]++; }

	// 위상 순서대로 연다: 맨손으로 남는 만큼 먼저 치고, 무기는 약한 것부터 써서 마지막 타격에 열리게 한다
	string out;
	char buf[32];
	for (int p = 0; p < N; p++) {
		int b = byOrd[p];
		vector<int> ws;
		for (int w = 0; w < N; w++) for (int u = 0; u < C[w]; u++) if (tgt[w][u] == b) ws.push_back(w);
		int bare = cst(R[b]);
		for (int i = 0; i < bare; i++) { snprintf(buf, sizeof(buf), "-1 %d\n", b); out += buf; }
		sort(ws.begin(), ws.end(), [&](int x, int y) { return A[x][b] < A[y][b]; });
		int h = H[b] - bare;
		for (int w : ws) {
			if (h <= 0) break;
			snprintf(buf, sizeof(buf), "%d %d\n", w, b); out += buf;
			h -= A[w][b];
		}
	}
	fputs(out.c_str(), stdout);
	return 0;
}
