#include<iostream>
#include<queue>

using namespace std;

int m, n, h, cnt;
int zdir[] = { 0,0,0,0,-1,1 };
int ydir[] = { 0,0,-1,1,0,0 };
int xdir[] = { -1,1,0,0,0,0 };
int box[100][100][100];

typedef struct Coord {
	int z, y, x;
}Coord;

bool isboundary(int k, int i,int j) {
	return (k >= 0) && (k < h) && (i >= 0) && (i < n) && (j >= 0) && (j < m);
}

queue<Coord> q;

void bfs() {
	cnt = -1;
	while (!q.empty()) {
		int size = q.size();
		cnt++;
		for (int i = 0; i < size; i++) {
			Coord coord = q.front(); q.pop();
			for (int d = 0; d < 6; d++) {
				if (isboundary(coord.z + zdir[d], coord.y + ydir[d], coord.x + xdir[d])) {
					if (box[coord.z + zdir[d]][coord.y + ydir[d]][coord.x + xdir[d]] == 0) {
						box[coord.z + zdir[d]][coord.y + ydir[d]][coord.x + xdir[d]] = 1;
						q.push({ coord.z + zdir[d], coord.y + ydir[d], coord.x + xdir[d] });
					}
				}
			}
		}
	}
}

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(NULL);
	cout.tie(NULL);

	cin >> m >> n >> h;
	for (int k = 0; k < h; k++) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < m; j++) {
				cin >> box[k][i][j];
				if (box[k][i][j] == 1) {
					q.push({ k,i,j });
				}
				else if (box[k][i][j] == 0) {
					cnt++;
				}
			}
		}
	}

	if (cnt == 0) {
		cout << 0 << "\n";
		return 0;
	}

	bfs();

	for (int k = 0; k < h; k++) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < m; j++) {
				if (box[k][i][j] == 0) {
					cout << -1 << "\n";
					return 0;
				}
			}
		}
	}

	cout << cnt << "\n";
	return 0;
}