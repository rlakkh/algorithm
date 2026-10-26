// Codeforces - 사전순 최소 부분배열 연산 op 를 반복 적용한 B_i[j] 질의 (문제 번호 미확인)
// 1) op(C) = "k번째 블록 = C[1..k] 의 접미사" n개를 이어붙인 문자열 중 사전순 최소의 앞 n글자.
//    그래서 op(C) 의 앞 k개는 C 의 앞 k개만으로 정해진다.
// 2) 블록 k 는 C[1..k] 의 린던 분해 마지막 인자 L(끝에 e번 반복)로 정해진다:
//    뒤에 이어질 문자열 S(k+1) < L^inf 이면 L 하나, 아니면 L^e 전체.
// 3) 뒤에서부터 블록을 정하고, S(k+1) 과 L^inf 의 LCP 는 다항 해시 + 지수 탐색으로 O(log n).
// 4) op 를 반복하면 증가가 시작되기 직전의 같은 값 묶음 길이가 g -> g(g+3)/2 로 커져서
//    n = 1e5 에서도 6~7번이면 고정점(비증가 배열)이 된다. 그때까지의 B 를 저장해 두고 답한다.
#include <bits/stdc++.h>
using namespace std;
typedef unsigned long long ull;
typedef long long ll;

const ull MOD = (1ULL << 61) - 1;
ull mulmod(ull a, ull b) {
	__uint128_t c = (__uint128_t)a * b;
	ull r = (ull)(c >> 61) + (ull)(c & MOD);
	return r >= MOD ? r - MOD : r;
}
ull addmod(ull a, ull b) { ull c = a + b; return c >= MOD ? c - MOD : c; }
ull submod(ull a, ull b) { return a >= b ? a - b : a + MOD - b; }
ull powmod(ull a, ull e) { ull r = 1; while (e) { if (e & 1) r = mulmod(r, a); a = mulmod(a, a); e >>= 1; } return r; }

int n;
vector<ull> pw, ipw, invgm;	// B^i, B^-i, 1/(B^l - 1)

// op(C): k번째 블록 = C[0..k-1] 의 접미사, 이어붙인 문자열의 사전순 최솟값의 앞 n글자
vector<int> op(const vector<int>& C) {
	vector<ull> pc(n + 1, 0);
	for (int i = 0; i < n; i++) pc[i + 1] = addmod(pc[i], mulmod((ull)C[i] + 1, pw[i]));
	auto sub = [&](int a, int len) -> ull { return mulmod(submod(pc[a + len], pc[a]), ipw[a]); };

	// M[J]: 접두사 C[0..J-1] 의 최소 접미사(마지막 린던 인자) 시작 - 뒤발 알고리즘
	vector<int> M(n + 1, 0);
	for (int i = 0; i < n;) {
		int j = i + 1, k = i;
		M[i + 1] = i;
		while (j < n && C[k] <= C[j]) {
			k = C[k] < C[j] ? i : k + 1;
			j++;
			int p = j - k, u = (j - i) % p;
			M[j] = u == 0 ? j - p : (j - u) + (M[i + u] - i);
		}
		while (i <= k) i += j - k;
	}
	// f[k]: 마지막 인자와 같은 인자가 연속으로 반복되는 구간의 시작
	vector<int> f(n + 1, 0);
	for (int k = 1; k <= n; k++) {
		int s = M[k], l = k - s, q = s;
		if (q >= 1 && q - M[q] == l && sub(M[q], l) == sub(s, l)) f[k] = f[q];
		else f[k] = s;
	}

	// 뒤에서부터 블록 결정. 블록 r = C[p[r] .. r-1]
	vector<int> p(n + 2, 0);
	vector<ll> Q(n + 2, 0);		// Q[t] = 블록 t..n 의 길이 합
	vector<ull> G(n + 2, 0);	// G[t] = 블록 t..n 을 이어붙인 문자열의 해시
	for (int r = n; r >= 1; r--) {
		int s = M[r], l = r - s, choose = s;
		int cap = n - r;
		if (f[r] != s && cap > 0) {
			ull Lh = sub(s, l);
			// L^w 의 [o, o+x) 구간 해시
			auto rotPre = [&](int o, int y) -> ull {
				if (y <= l - o) return sub(s + o, y);
				return addmod(sub(s + o, l - o), mulmod(pw[l - o], sub(s, y - (l - o))));
			};
			auto perHash = [&](ll o0, int x) -> ull {
				int o = (int)(o0 % l);
				ull R = o == 0 ? Lh : rotPre(o, l);
				int q = x / l, rem = x % l;
				ull geo = mulmod(submod(pw[q * l], 1), invgm[l]);
				return addmod(mulmod(R, geo), mulmod(pw[q * l], rotPre(o, rem)));
			};
			auto matchBlocks = [&](int t) -> bool {
				ll len = Q[r + 1] - Q[t];
				if (len > cap) return false;
				ull h = submod(G[r + 1], mulmod(pw[len], G[t]));
				return h == perHash(0, (int)len);
			};
			int lo = r + 1, step = 1;
			while (lo + step <= n + 1 && matchBlocks(lo + step)) { lo += step; step <<= 1; }
			int hi = min(lo + step, n + 2);
			while (hi - lo > 1) { int mid = (lo + hi) / 2; if (matchBlocks(mid)) lo = mid; else hi = mid; }
			int t = lo;
			ll m0 = Q[r + 1] - Q[t];
			if (t <= n && m0 < cap) {
				int pt = p[t], bl = t - pt;
				int lim = (int)min<ll>(bl, cap - m0);
				auto matchIn = [&](int x) -> bool { return sub(pt, x) == perHash(m0, x); };
				int xl = 0, st = 1;
				while (xl + st <= lim && matchIn(xl + st)) { xl += st; st <<= 1; }
				int xh = min(xl + st, lim + 1);
				while (xh - xl > 1) { int mid = (xl + xh) / 2; if (matchIn(mid)) xl = mid; else xh = mid; }
				ll lcp = m0 + xl;
				if (lcp < cap && C[pt + xl] > C[s + (int)(lcp % l)]) choose = f[r];
			}
		}
		p[r] = choose;
		int blen = r - choose;
		Q[r] = Q[r + 1] + blen;
		G[r] = addmod(sub(choose, blen), mulmod(pw[blen], G[r + 1]));
	}
	vector<int> out;
	out.reserve(n);
	for (int r = 1; r <= n && (int)out.size() < n; r++)
		for (int i = p[r]; i < r && (int)out.size() < n; i++) out.push_back(C[i]);
	return out;
}

static char ibuf[1 << 25];
int ipos = 0, ilen = 0;
inline int readChar() {
	if (ipos == ilen) { ilen = (int)fread(ibuf, 1, sizeof(ibuf), stdin); ipos = 0; if (ilen <= 0) return -1; }
	return ibuf[ipos++];
}
inline int readInt() {
	int c = readChar();
	while (c != '-' && (c < '0' || c > '9')) c = readChar();
	int x = 0;
	while (c >= '0' && c <= '9') { x = x * 10 + (c - '0'); c = readChar(); }
	return x;
}

int main() {
	n = readInt();
	vector<int> A(n);
	for (int i = 0; i < n; i++) A[i] = readInt();
	mt19937_64 rng(chrono::steady_clock::now().time_since_epoch().count());
	ull B = rng() % (MOD - 1000) + 500;
	pw.assign(n + 2, 1); ipw.assign(n + 2, 1); invgm.assign(n + 2, 0);
	ull Binv = powmod(B, MOD - 2);
	for (int i = 1; i <= n + 1; i++) { pw[i] = mulmod(pw[i - 1], B); ipw[i] = mulmod(ipw[i - 1], Binv); }
	for (int l = 1; l <= n; l++) invgm[l] = powmod(submod(pw[l], 1), MOD - 2);

	vector<vector<int>> Bs;
	Bs.push_back(A);
	while (true) {
		vector<int> nb = op(Bs.back());
		if (nb == Bs.back()) break;
		Bs.push_back(move(nb));
	}
	int q = readInt();
	string out;
	out.reserve((size_t)q * 7);
	char tmp[12];
	int last = (int)Bs.size() - 1;
	for (int t = 0; t < q; t++) {
		int i = readInt(), j = readInt();
		int v = Bs[min(i, last)][j - 1];
		int len = 0;
		do { tmp[len++] = char('0' + v % 10); v /= 10; } while (v);
		while (len) out.push_back(tmp[--len]);
		out.push_back('\n');
	}
	fwrite(out.data(), 1, out.size(), stdout);
	return 0;
}
