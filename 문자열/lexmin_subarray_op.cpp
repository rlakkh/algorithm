#include <cstdio>
#include <vector>
#include <algorithm>

using namespace std;

int n;
char ibuf[1 << 25];
int ipos, ilen;
char obuf[1 << 23];
int opos;

int readChar() {
	if (ipos == ilen) {
		ilen = fread(ibuf, 1, sizeof(ibuf), stdin);
		ipos = 0;
		if (ilen <= 0) return -1;
	}
	return ibuf[ipos++];
}

int readInt() {
	int c = readChar();
	while (c < '0' || c > '9') c = readChar();
	int x = 0;
	while (c >= '0' && c <= '9') {
		x = x * 10 + (c - '0');
		c = readChar();
	}
	return x;
}

void writeInt(int v) {
	char tmp[12];
	int len = 0;
	do {
		tmp[len++] = '0' + v % 10;
		v /= 10;
	} while (v);
	while (len) obuf[opos++] = tmp[--len];
	obuf[opos++] = '\n';
}

vector<int> op(const vector<int>& C) {
	vector<int> M(n + 1), F(n + 1), P(n + 1), dec(n + 2, 0);
	int i = 0;
	while (i < n) {
		int j = i + 1, k = i;
		M[i + 1] = i;
		F[i + 1] = i;
		while (j < n && C[k] <= C[j]) {
			k = C[k] < C[j] ? i : k + 1;
			j++;
			int p = j - k, u = (j - i) % p;
			if (u == 0) {
				M[j] = j - p;
				F[j] = i;
			}
			else {
				M[j] = j - u + M[i + u] - i;
				F[j] = j - u + F[i + u] - i;
			}
		}
		while (i <= k) i += j - k;
	}
	for (int r = n; r >= 1; r--) {
		int s = M[r], f = F[r];
		if (f == s) {
			P[r] = s;
			continue;
		}
		int mu = C[s];
		int all;
		if (r == n || C[r] < mu) all = 0;
		else if (C[r] > mu) all = 1;
		else if (r - s > 1) all = 0;
		else all = dec[r + 1];
		dec[r] = all;
		P[r] = all ? f : s;
	}
	vector<int> out(n);
	int cnt = 0;
	for (int r = 1; r <= n && cnt < n; r++)
		for (int x = P[r]; x < r && cnt < n; x++) out[cnt++] = C[x];
	return out;
}

int main() {
	n = readInt();
	vector<int> A(n);
	for (int i = 0; i < n; i++) A[i] = readInt();
	vector<vector<int>> B;
	B.push_back(A);
	while (true) {
		vector<int> nb = op(B.back());
		if (nb == B.back()) break;
		B.push_back(nb);
	}
	int last = (int)B.size() - 1;
	int q = readInt();
	while (q--) {
		int i = readInt(), j = readInt();
		writeInt(B[min(i, last)][j - 1]);
	}
	fwrite(obuf, 1, opos, stdout);
	return 0;
}
