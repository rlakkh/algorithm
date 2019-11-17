#define _CRT_SECURE_NO_WARNINGS
#include<iostream>

using namespace std;

int dp[501][501];
int map[500][500];
int n, m;
int xdir[] = { 0,0,-1,1 };
int ydir[] = { -1,1,0,0 };

bool isBoundary(int i, int j) {
	return (i < n) && (i >= 0) && (j < m) && (j >= 0);
}

int cntPath(int i, int j) {
	if (dp[i][j]!=-1) return dp[i][j];
	if (i == 0 && j == 0) {
		return dp[i][j] = 1;
	}
	else {
		int cnt = 0;
		for (int d = 0; d < 4; d++) {
			if (isBoundary(i + ydir[d], j + xdir[d])) {
				if (map[i][j] < map[i + ydir[d]][j + xdir[d]]) {
					cnt += cntPath(i + ydir[d], j + xdir[d]);
				}
			}
		}
		return dp[i][j] = cnt;
	}
	
}

int main()
{
	scanf("%d %d", &n, &m);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			scanf("%d", &map[i][j]);
			dp[i][j] = -1;
		}
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			cntPath(i, j);
		}
	}

	printf("%d\n", dp[n-1][m-1]);
	return 0;
}