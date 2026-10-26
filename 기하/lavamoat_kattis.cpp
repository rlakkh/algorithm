// Kattis lavamoat - Lava Moat
// 1) 삼각형 안의 등고선 조각은 모두 평행해서 길이가 h 에 대해 1차함수 -> 꼭짓점 고도만 확인하면 된다.
// 2) 남쪽이 낮은 방향이면 서남~동남이 {z<h}, 서북~동북이 {z>h} 에서 이어질 때만 서->동 등고선이 생긴다 -> 구간 [hD, hU].
//    서/동 변의 방향이 엇갈리면 한 꼭짓점에서 꺾이는 고도 하나뿐이라 후보 4개를 직접 확인한다.
// 3) 단위 기울기장 F = grad z / |grad z| 에 발산 정리: 등고선 길이 = sum K_e * (변 e 중 남쪽 영역 R(h) 에 든 비율).
//    R(h) 는 h 에 대해 커지기만 하고, 각 점이 R 에 들어가는 시점 d 를 크루스칼 재구성 트리로 구해 이벤트 스윕을 한다.
// 4) 안장점 고도에서는 남쪽에 매달린 고리(플럭스가 음수인 덩어리)를 건너뛰는 더 짧은 경로가 생긴다.
// 5) 스윕 값으로 후보를 고른 뒤 가장 좋은 몇 개 고도는 등고선 그래프 다익스트라로 정확히 다시 잰다.
#include <bits/stdc++.h>
using namespace std;
typedef long long ll;
typedef long double ld;

const ld INF = 1e30L;
int n, m, E;
vector<ll> X, Y, Z;
vector<array<int, 3>> tri, triE;
vector<int> eu, ev;
int SW, SE, NE, NW, edgeW, edgeE;

struct DSU {
	vector<int> p;
	void init(int k) { p.resize(k); iota(p.begin(), p.end(), 0); }
	int f(int x) { while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; } return x; }
};

// 크루스칼 재구성 트리를 만들고, 리프 s 와의 LCA 레벨을 모든 리프에 대해 구한다
vector<ll> lcaLevelWith(int s, const vector<ll>& leafLev, const vector<ll>& w, bool ascending) {
	vector<int> order(E);
	iota(order.begin(), order.end(), 0);
	if (ascending) sort(order.begin(), order.end(), [&](int a, int b) { return w[a] < w[b]; });
	else sort(order.begin(), order.end(), [&](int a, int b) { return w[a] > w[b]; });
	int tot = 2 * n - 1;
	vector<int> par(tot, -1);
	vector<ll> lev(tot);
	for (int i = 0; i < n; i++) lev[i] = leafLev[i];
	DSU d; d.init(tot);
	int cnt = n;
	for (int e : order) {
		int a = d.f(eu[e]), b = d.f(ev[e]);
		if (a == b) continue;
		par[a] = par[b] = cnt; lev[cnt] = w[e];
		d.p[a] = d.p[b] = cnt;
		cnt++;
	}
	vector<char> mark(tot, 0);
	for (int x = s; x != -1; x = par[x]) mark[x] = 1;
	vector<ll> val(tot);
	for (int x = cnt - 1; x >= 0; x--) val[x] = mark[x] ? lev[x] : val[par[x]];
	val.resize(n);
	return val;
}

// 고도 h 의 등고선 그래프에서 서쪽 경계점 -> 동쪽 경계점 최단 거리
ld directTrace(ll h, const vector<ll>& zz) {
	auto sg = [&](int i) { return (zz[i] > h) - (zz[i] < h); };
	auto pos = [&](int node) -> pair<ld, ld> {
		if (node >= E) return {(ld)X[node - E], (ld)Y[node - E]};
		int a = eu[node], b = ev[node];
		ld f = (ld)(h - zz[a]) / (ld)(zz[b] - zz[a]);
		return {X[a] + f * (X[b] - X[a]), Y[a] + f * (Y[b] - Y[a])};
	};
	auto onEdge = [&](int e, int a, int b) -> int {
		if (zz[a] == h) return E + a;
		if (zz[b] == h) return E + b;
		if ((zz[a] < h) != (zz[b] < h)) return e;
		return -1;
	};
	int src = onEdge(edgeW, SW, NW), dst = onEdge(edgeE, SE, NE);
	if (src < 0 || dst < 0) return INF;
	vector<vector<pair<int, ld>>> adj(E + n);
	for (int t = 0; t < m; t++) {
		int s[3], cnt = 0, nd[3];
		for (int k = 0; k < 3; k++) s[k] = sg(tri[t][k]);
		for (int k = 0; k < 3; k++) if (s[k] == 0) nd[cnt++] = E + tri[t][k];
		for (int k = 0; k < 3; k++) if (s[k] * s[(k + 1) % 3] < 0) nd[cnt++] = triE[t][k];
		if (cnt != 2) continue;
		auto p = pos(nd[0]), q = pos(nd[1]);
		ld len = hypotl(p.first - q.first, p.second - q.second);
		adj[nd[0]].push_back({nd[1], len});
		adj[nd[1]].push_back({nd[0], len});
	}
	vector<ld> dist(E + n, INF);
	priority_queue<pair<ld, int>, vector<pair<ld, int>>, greater<pair<ld, int>>> pq;
	dist[src] = 0; pq.push({0, src});
	while (!pq.empty()) {
		auto [d, u] = pq.top(); pq.pop();
		if (d > dist[u]) continue;
		if (u == dst) return d;
		for (auto [v, c] : adj[u]) if (dist[v] > d + c) { dist[v] = d + c; pq.push({dist[v], v}); }
	}
	return INF;
}

ld solve(const vector<ll>& zz) {
	// c: 서북 모서리와 고도 > h 로 연결된 마지막 h (위쪽 집합의 병목값)
	vector<ll> wmin(E), wmax(E);
	for (int e = 0; e < E; e++) { wmin[e] = min(zz[eu[e]], zz[ev[e]]); wmax[e] = max(zz[eu[e]], zz[ev[e]]); }
	vector<ll> c = lcaLevelWith(NW, zz, wmin, false);
	ll hU = c[NE];
	ll hD = lcaLevelWith(SW, zz, wmax, true)[SE];
	if (hD > hU) return INF;
	if (hD == hU) return directTrace(hD, zz);
	// d: 남쪽 영역 R(h) 에 들어가는 시점 (c 기준 최소 병목)
	vector<ll> wc(E);
	for (int e = 0; e < E; e++) wc[e] = max(c[eu[e]], c[ev[e]]);
	vector<ll> d = lcaLevelWith(SW, c, wc, true);

	// K[e] = 변 e 에 대한 단위 기울기장 플럭스 계수 (변 전체가 R 안에 있을 때의 기여)
	vector<ld> K(E, 0);
	for (int t = 0; t < m; t++) {
		int p0 = tri[t][0], p1 = tri[t][1], p2 = tri[t][2];
		ld nx = (ld)((Y[p1] - Y[p0]) * (zz[p2] - zz[p0]) - (zz[p1] - zz[p0]) * (Y[p2] - Y[p0]));
		ld ny = (ld)((zz[p1] - zz[p0]) * (X[p2] - X[p0]) - (X[p1] - X[p0]) * (zz[p2] - zz[p0]));
		ld gx = -nx, gy = -ny, gl = sqrtl(gx * gx + gy * gy);
		for (int k = 0; k < 3; k++) {
			int a = tri[t][k], b = tri[t][(k + 1) % 3];
			ld Nx = (ld)(Y[b] - Y[a]), Ny = (ld)(X[a] - X[b]);
			K[triE[t][k]] -= (gx * Nx + gy * Ny) / gl;
		}
	}

	DSU comp; comp.init(n);
	for (int e = 0; e < E; e++) {
		int a = eu[e], b = ev[e];
		if (d[a] == d[b] && zz[a] != d[a] && zz[b] != d[b]) comp.p[comp.f(a)] = comp.f(b);
	}

	struct Ev { ll t; int e, kind; };
	vector<Ev> evs;
	ld V = 0, S = 0;
	for (int e = 0; e < E; e++) {
		int a = eu[e], b = ev[e];
		if (zz[a] > zz[b]) swap(a, b);
		ld k = K[e], dz = (ld)(zz[b] - zz[a]);
		if (c[b] <= zz[a] || d[a] >= c[b]) {
			if (d[a] <= hD) V += k;
			else if (d[a] < hU) evs.push_back({d[a], e, 0});
		} else if (d[a] <= hD) {
			if (c[b] <= hD) V += k;
			else {
				V += k * (ld)(hD - zz[a]) / dz; S += k / dz;
				if (c[b] < hU) evs.push_back({c[b], e, 2});
			}
		} else if (d[a] < hU) {
			evs.push_back({d[a], e, 1});
			if (c[b] < hU) evs.push_back({c[b], e, 2});
		}
	}
	sort(evs.begin(), evs.end(), [](const Ev& p, const Ev& q) { return p.t < q.t; });

	ld best = min(directTrace(hD, zz), directTrace(hU, zz));
	vector<pair<ld, ll>> cand;	// (스윕으로 구한 값, 고도) - 최종값은 직접 추적으로 다시 계산
	vector<ld> acc(n, 0);
	vector<char> inList(n, 0);
	vector<int> touched;
	ll hcur = hD;
	for (size_t i = 0; i < evs.size();) {
		ll t = evs[i].t;
		V += S * (ld)(t - hcur); hcur = t;
		ld Lminus = V;
		touched.clear();
		for (; i < evs.size() && evs[i].t == t; i++) {
			int e = evs[i].e, a = eu[e], b = ev[e];
			if (zz[a] > zz[b]) swap(a, b);
			ld k = K[e], dz = (ld)(zz[b] - zz[a]), jump, ds = 0;
			int attr;
			if (evs[i].kind == 0) { jump = k; attr = zz[a] != d[a] ? a : b; }
			else if (evs[i].kind == 1) { jump = k * (ld)(t - zz[a]) / dz; ds = k / dz; attr = zz[a] != d[a] ? a : b; }
			else { jump = k * (ld)(zz[b] - t) / dz; ds = -k / dz; attr = b; }
			V += jump; S += ds;
			int r = comp.f(attr);
			if (!inList[r]) { inList[r] = 1; acc[r] = 0; touched.push_back(r); }
			acc[r] += jump;
		}
		// 안장점에서는 남쪽으로 매달린 고리(음의 플럭스)를 건너뛴 경로가 가능
		ld Ld = Lminus;
		for (int r : touched) { Ld += min(acc[r], (ld)0); inList[r] = 0; }
		cand.push_back({Ld, t});
	}
	// 누적 오차를 피하려고 값이 작은 몇 개의 고도만 다익스트라로 정확히 다시 잰다
	int keep = min((int)cand.size(), 6);
	partial_sort(cand.begin(), cand.begin() + keep, cand.end());
	for (int i = 0; i < keep; i++) best = min(best, directTrace(cand[i].second, zz));
	return best;
}

int main() {
	int T;
	if (scanf("%d", &T) != 1) return 0;
	while (T--) {
		ll w, l;
		scanf("%lld %lld %d %d", &w, &l, &n, &m);
		X.assign(n, 0); Y.assign(n, 0); Z.assign(n, 0);
		for (int i = 0; i < n; i++) scanf("%lld %lld %lld", &X[i], &Y[i], &Z[i]);
		tri.assign(m, {}); triE.assign(m, {});
		vector<pair<ll, int>> keys;
		for (int t = 0; t < m; t++) {
			for (int k = 0; k < 3; k++) { scanf("%d", &tri[t][k]); tri[t][k]--; }
			for (int k = 0; k < 3; k++) {
				int a = tri[t][k], b = tri[t][(k + 1) % 3];
				keys.push_back({(ll)min(a, b) * n + max(a, b), t * 3 + k});
			}
		}
		sort(keys.begin(), keys.end());
		eu.clear(); ev.clear(); E = 0;
		for (size_t i = 0; i < keys.size(); i++) {
			if (i == 0 || keys[i].first != keys[i - 1].first) {
				eu.push_back(keys[i].first / n); ev.push_back(keys[i].first % n); E++;
			}
			triE[keys[i].second / 3][keys[i].second % 3] = E - 1;
		}
		for (int i = 0; i < n; i++) {
			if (X[i] == 0 && Y[i] == 0) SW = i;
			if (X[i] == w && Y[i] == 0) SE = i;
			if (X[i] == w && Y[i] == l) NE = i;
			if (X[i] == 0 && Y[i] == l) NW = i;
		}
		edgeW = edgeE = -1;
		for (int e = 0; e < E; e++) {
			if (min(eu[e], ev[e]) == min(SW, NW) && max(eu[e], ev[e]) == max(SW, NW)) edgeW = e;
			if (min(eu[e], ev[e]) == min(SE, NE) && max(eu[e], ev[e]) == max(SE, NE)) edgeE = e;
		}
		ld ans = solve(Z);
		vector<ll> negZ(n);
		for (int i = 0; i < n; i++) negZ[i] = -Z[i];
		ans = min(ans, solve(negZ));
		// 서쪽/동쪽 변의 방향이 엇갈리는 경우: 한 꼭짓점에서 방향이 바뀌는 고도 하나뿐
		vector<ll> wmin(E), wmax(E);
		for (int e = 0; e < E; e++) { wmin[e] = min(Z[eu[e]], Z[ev[e]]); wmax[e] = max(Z[eu[e]], Z[ev[e]]); }
		vector<ll> subSW = lcaLevelWith(SW, Z, wmax, true), subNW = lcaLevelWith(NW, Z, wmax, true);
		vector<ll> supSW = lcaLevelWith(SW, Z, wmin, false), supNW = lcaLevelWith(NW, Z, wmin, false);
		for (ll h : {subSW[NE], supNW[SE], subNW[SE], supSW[NE]}) ans = min(ans, directTrace(h, Z));
		if (ans >= INF / 2) printf("impossible\n");
		else printf("%.9Lf\n", ans);
	}
	return 0;
}
