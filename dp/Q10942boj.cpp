#define _CRT_SECURE_NO_WARNINGS
#include<iostream>

using namespace std;

int n;
int nums[2001];
int dp[2001][2001];
int q;

int main()
{
	scanf("%d", &n);

	for (int i = 1; i <= n; i++) {
		scanf("%d", &nums[i]);
		dp[i][i] = 1;
	}
	
	for (int i = 1; i < n; i++) {
		for (int s = 1; s <= n; s++) {
			if (nums[s] == nums[s + i]) {
				if (i == 1) {
					dp[s][s + i] = 1;
				}
				else {
					if (dp[s + 1][s + i - 1]) dp[s][s + i] = 1;
				}
			}
		}
	}

	scanf("%d", &q);

	while (q--) {
		int s, e;
		scanf("%d %d", &s, &e);
		printf("%d\n", dp[s][e]);
	}

	return 0;
}