#include <string>
#include <vector>
#include <queue>

using namespace std;

typedef struct {
	int p; int l;
}Work;

int solution(vector<int> priorities, int location) {
	int answer = 0;
	int count[10] = { 0, };
	int size = priorities.size();
	int n = 9, index = 0;
	queue<Work> q;
	for (int i = 0; i < size; i++)
		count[priorities[i]]++;
	while (n > 0) {
		while (count[n] == 0 && n > 0) n--;
		if (n == priorities[index % size]) {
			q.push({ n,index % size });
			count[n]--;
		}
		index++;
	}
	Work tmp;
	do {
		tmp = q.front(); q.pop();
		answer++;
	} while (tmp.l != location);
	return answer;
}