#include<iostream>
#include<vector>
#include<algorithm>
#include<random>
#include<chrono>

using namespace std;

/*
 * 점(경계) 0..n 을 원 위에 놓고, 감지기 [l, r] 을 점 l-1 과 r 을 잇는 현으로 본다.
 * 칸 i 는 점 i-1 과 i 사이의 틈이고, 칸 i 의 응답 = i 를 가로지르는 현들의 집합.
 * 응답이 같은 칸끼리 묶은 클래스들은 서로 교차하지 않는다(non-crossing).
 *
 * 크기 s>=2 인 클래스는 원을 s 개의 호(그룹)로 나누고, 새 현의 끝점이 모든 그룹에 있어야 한다.
 * 다른 그룹을 포함하지 않는 "최소 그룹"들은 서로 소이므로, 그 개수 H 에 대해 답 >= ceil(H/2).
 * 최소 그룹 = 크기 1 인 클래스를 지운 칸들의 원형 수열에서, 인접한 두 칸이 같은 클래스인 경우.
 * 각 최소 그룹에서 점 하나씩 골라 정렬한 뒤 x[i] 와 x[i + K/2] 를 이으면 ceil(H/2) 개로 충분하다.
 */

typedef unsigned long long ull;

int main()
{
	ios::sync_with_stdio(false);
	cin.tie(nullptr);

	mt19937_64 rng(chrono::steady_clock::now().time_since_epoch().count());

	int t;
	cin >> t;
	while (t--) {
		int n, m;
		cin >> n >> m;

		vector<ull> diff(n + 2, 0);
		for (int i = 0; i < m; i++) {
			int l, r;
			cin >> l >> r;
			ull v = rng();
			diff[l] ^= v;
			diff[r + 1] ^= v;
		}

		// sig[i] : 칸 i 를 덮는 감지기 집합의 해시
		vector<ull> sig(n + 1, 0);
		for (int i = 1; i <= n; i++) sig[i] = sig[i - 1] ^ diff[i];

		vector<ull> sorted_sig(sig.begin() + 1, sig.end());
		sort(sorted_sig.begin(), sorted_sig.end());
		vector<int> cls(n + 1), cnt(n, 0);
		for (int i = 1; i <= n; i++) {
			cls[i] = lower_bound(sorted_sig.begin(), sorted_sig.end(), sig[i]) - sorted_sig.begin();
			cnt[cls[i]]++;
		}

		vector<int> spots;
		for (int i = 1; i <= n; i++) {
			if (cnt[cls[i]] >= 2) spots.push_back(i);
		}

		// 최소 그룹마다 점 하나 (그룹을 여는 칸 a 의 바로 오른쪽 점 a)
		vector<int> pts;
		int L = spots.size();
		for (int i = 0; i < L; i++) {
			if (cls[spots[i]] == cls[spots[(i + 1) % L]]) pts.push_back(spots[i]);
		}

		int H = pts.size();
		if (H % 2 == 1) pts.insert(pts.begin(), pts[0]);
		int K = pts.size();

		cout << K / 2 << "\n";
		for (int i = 0; i < K / 2; i++) {
			// 현 (pts[i], pts[i + K/2]) -> 구간 [pts[i] + 1, pts[i + K/2]]
			cout << pts[i] + 1 << " " << pts[i + K / 2] << "\n";
		}
	}

	return 0;
}
