#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<algorithm>

using namespace std;

/*
 * 길이 L 인 패턴의 등장 위치를 p_1 < ... < p_c, 간격을 g_j = p_{j+1} - p_j 라 하자.
 * j 번째 적용을 취소했을 때의 손실은
 *     max(0, min(L, 왼쪽 간격) + min(L, 오른쪽 간격) - L)   (이웃이 없으면 그쪽 간격은 무한대)
 * 따라서
 *  - L <= k 이면 항상 resilient.
 *  - L >  k 이면 c >= 2, g_1 <= k, g_{c-1} <= k 이어야 하고,
 *    양쪽 간격이 모두 k 보다 큰 가운데 등장 (a, x, b) 마다 L >= b - a - k 이어야 한다.
 *
 * SAM 의 한 상태에 속한 부분문자열들은 endpos 집합(= 간격)이 같고 길이만 (len(link), len] 으로 다르다.
 * 상태마다 첫 간격, 마지막 간격, T = max(b - a - k) 를 알면 조건을 만족하는 길이의 개수를 바로 센다.
 * endpos 집합은 suffix link 트리를 따라 세그먼트 트리 합치기로 관리한다.
 */

typedef long long ll;

const int INF = 1 << 30;

int n, k;
// 노드마다 집합의 가장 작은 두 원소(mn1, mn2), 가장 큰 두 원소(mx1, mx2), 내부의 T(best)
struct Node {
	int lc, rc, mn1, mn2, mx1, mx2, best;
};
vector<Node> tr;
int node_cnt;

void pull(int x) {
	Node& t = tr[x];
	if (!t.lc || !t.rc) {
		const Node& c = tr[t.lc ? t.lc : t.rc];
		t.mn1 = c.mn1; t.mn2 = c.mn2;
		t.mx1 = c.mx1; t.mx2 = c.mx2;
		t.best = c.best;
		return;
	}
	const Node& L = tr[t.lc];
	const Node& R = tr[t.rc];
	t.mn1 = L.mn1;
	t.mn2 = (L.mn2 != INF) ? L.mn2 : R.mn1;
	t.mx1 = R.mx1;
	t.mx2 = (R.mx2 != 0) ? R.mx2 : L.mx1;

	// 경계를 사이에 둔 두 원소 a < c 가 새로 이웃이 된다
	int b = max(L.best, R.best);
	int a = L.mx1, c = R.mn1;
	if (c - a > k) {
		if (L.mx2 != 0 && a - L.mx2 > k) b = max(b, c - L.mx2 - k);
		if (R.mn2 != INF && R.mn2 - c > k) b = max(b, R.mn2 - a - k);
	}
	t.best = b;
}

int insert(int x, int l, int r, int p) {
	if (!x) x = ++node_cnt;
	if (l == r) {
		tr[x] = { 0, 0, p, INF, p, 0, 0 };
		return x;
	}
	int m = (l + r) / 2;
	if (p <= m) {
		int c = insert(tr[x].lc, l, m, p);
		tr[x].lc = c;
	}
	else {
		int c = insert(tr[x].rc, m + 1, r, p);
		tr[x].rc = c;
	}
	pull(x);
	return x;
}

int unite(int x, int y, int l, int r) {
	if (!x || !y) return x | y;
	if (l == r) return x;
	int m = (l + r) / 2;
	int a = unite(tr[x].lc, tr[y].lc, l, m);
	int b = unite(tr[x].rc, tr[y].rc, m + 1, r);
	tr[x].lc = a;
	tr[x].rc = b;
	pull(x);
	return x;
}

int main()
{
	ios::sync_with_stdio(false);
	cin.tie(nullptr);

	string s;
	cin >> n >> k >> s;

	int maxStates = 2 * n + 1;
	vector<array<int, 26>> nxt(maxStates);
	vector<int> link(maxStates, -1), len(maxStates, 0), pos(maxStates, 0);
	int sz = 1, last = 0;
	for (int i = 0; i < n; i++) {
		int c = s[i] - 'a';
		int cur = sz++;
		len[cur] = len[last] + 1;
		pos[cur] = i + 1;
		int p = last;
		while (p != -1 && !nxt[p][c]) {
			nxt[p][c] = cur;
			p = link[p];
		}
		if (p == -1) link[cur] = 0;
		else {
			int q = nxt[p][c];
			if (len[p] + 1 == len[q]) link[cur] = q;
			else {
				int clone = sz++;
				len[clone] = len[p] + 1;
				nxt[clone] = nxt[q];
				link[clone] = link[q];
				while (p != -1 && nxt[p][c] == q) {
					nxt[p][c] = clone;
					p = link[p];
				}
				link[q] = link[cur] = clone;
			}
		}
		last = cur;
	}
	vector<array<int, 26>>().swap(nxt);

	// 길이 오름차순 정렬 (자식이 부모보다 먼저 처리되도록 뒤에서부터 순회)
	vector<int> bucket(n + 2, 0), order(sz);
	for (int v = 0; v < sz; v++) bucket[len[v]]++;
	for (int i = 1; i <= n; i++) bucket[i] += bucket[i - 1];
	for (int v = sz - 1; v >= 0; v--) order[--bucket[len[v]]] = v;

	int depth = 1;
	while ((1 << (depth - 1)) < n) depth++;
	size_t cap = (size_t)n * depth + 1;
	tr.assign(cap, Node{ 0, 0, 0, 0, 0, 0, 0 });

	vector<int> root(sz, 0);
	ll ans = 0;
	for (int idx = sz - 1; idx >= 1; idx--) {
		int v = order[idx];
		if (pos[v]) root[v] = insert(root[v], 1, n, pos[v]);

		int lo = len[link[v]], hi = len[v];
		ans += max(0, min(hi, k) - lo);

		int x = root[v];
		const Node& t = tr[x];
		if (t.mn2 != INF && t.mn2 - t.mn1 <= k && t.mx1 - t.mx2 <= k) {
			int from = max({ lo, k, t.best - 1 });
			if (hi > from) ans += hi - from;
		}

		if (link[v] > 0) root[link[v]] = unite(root[link[v]], x, 1, n);
	}

	cout << ans << "\n";

	return 0;
}
