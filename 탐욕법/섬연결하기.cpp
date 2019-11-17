#include <string>
#include <vector>
#include <string>
#include <algorithm>
#define V 100
#define E 4910

using namespace std;


int uf[V];

struct list {
	int x, y, cost;
}v[E];

int disjoint(int x) {
	return (x != uf[x]) ? uf[x] = disjoint(uf[x]) : x;
}

bool cmp(vector<int> v1, vector<int> v2){
    return v1[2]<v2[2];
}

int solution(int n, vector<vector<int>> costs) {
    int answer = 0;
    for (int i = 0; i < n; i++) uf[i] = i;
    sort(costs.begin(), costs.end(),cmp);
    for(int i=0;i<costs.size();i++){
        v[i].cost=costs[i][2];
        v[i].x=costs[i][1];
        v[i].y=costs[i][0];
    }
    
    for (int i = 0; i <costs.size(); i++)
		if (disjoint(v[i].y) != disjoint(v[i].x)) {
			uf[disjoint(v[i].y)] = uf[v[i].x];
			answer += v[i].cost;
		}
    return answer;
}