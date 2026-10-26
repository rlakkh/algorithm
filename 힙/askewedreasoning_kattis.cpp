// Kattis askewedreasoning - A-Skew-ed Reasoning
// 노드 v 의 서브트리에 들어간 값들의 삽입 순서 seq(v) 는 v 가 들어간 뒤 오른쪽/왼쪽 서브트리에 번갈아 들어가므로
// 왼쪽 크기 a, 오른쪽 크기 b 에 대해 두 가지 패턴(E, O)만 가능하다.
// 왼쪽 자식 없이 오른쪽 자식만 있으면 impossible, b 가 a-1 또는 a 일 때만 두 패턴 중 선택 가능.
// 사전순 최소는 v 가 맨 앞에 오는 패턴, 최대는 v 가 두 번째에 오는 패턴을 고르면 된다.
#include <cstdio>
#include <vector>
#include <utility>

using namespace std;

const int MAX = 200001;
int n;
int lc[MAX], rc[MAX], sz[MAX];
vector<int> seq[MAX];	// seq[0] 은 항상 빈 시퀀스 (자식이 없는 경우)
vector<int> tmp;

// E: sL[0..a-b-1], v, sR[0], sL[a-b], sR[1], sL[a-b+1], ..., sR[b-1], sL[a-1]   (a >= b)
// O: sR[0..b-a],   v, sL[0], sR[b-a+1], sL[1], ..., sR[b-1], sL[a-1]           (a >= 1, b >= a-1)
// 앞부분은 그대로 두고 뒷부분만 다시 쓰므로 O(min(a, b) + 1), 전체 O(n log n)
vector<int> mergeSeq(vector<int>& sL, vector<int>& sR, int v, bool useE) {
	int a = sL.size(), b = sR.size();
	vector<int>& first = useE ? sR : sL;
	vector<int> base(move(useE ? sL : sR));
	int k = useE ? a - b : b - a + 1;
	tmp.assign(base.begin() + k, base.end());
	base.resize(k);
	base.push_back(v);
	for (int i = 0; i < (int)first.size(); i++) {
		base.push_back(first[i]);
		if (i < (int)tmp.size()) base.push_back(tmp[i]);
	}
	vector<int>().swap(first);
	return base;
}

void solve(bool wantMin) {
	for (int v = n; v >= 1; v--) {
		int a = sz[lc[v]], b = sz[rc[v]];
		bool useE;
		if (a == 0 || b <= a - 2) useE = true;
		else if (b >= a + 1) useE = false;
		else if (b == a) useE = wantMin;	// E 가 v 를 맨 앞에 둔다
		else useE = !wantMin;				// b == a - 1 : O 가 v 를 맨 앞에 둔다
		seq[v] = mergeSeq(seq[lc[v]], seq[rc[v]], v, useE);
	}
	for (int i = 0; i < n; i++) printf("%d%c", seq[1][i], i + 1 < n ? ' ' : '\n');
	vector<int>().swap(seq[1]);
}

int main()
{
	scanf("%d", &n);
	for (int i = 1; i <= n; i++) scanf("%d %d", &lc[i], &rc[i]);
	for (int i = 1; i <= n; i++) {
		if (lc[i] == 0 && rc[i] != 0) {
			printf("impossible\n");
			return 0;
		}
	}
	for (int i = n; i >= 1; i--) sz[i] = 1 + sz[lc[i]] + sz[rc[i]];
	solve(true);
	solve(false);
	return 0;
}
