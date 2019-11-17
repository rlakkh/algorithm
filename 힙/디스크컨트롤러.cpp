#include <string>
#include <vector>
#include <queue>
#include <algorithm>

using namespace std;

struct cmp {
	bool operator()(vector<int> j1, vector<int> j2) {
		return j1[1] > j2[1];
	}
};

bool compare(vector<int> v1, vector<int> v2) {
	if (v1[0] < v2[0]) {
		return true;
	}
	else if (v1[0] == v2[0]) {
		if (v1[1] < v2[1]) return true;
	}
	return false;
}

int solution(vector<vector<int>> jobs) {
	int answer = 0, lefted = 0, time = 0;
	int size = jobs.size();
	int i = 1;
	priority_queue<vector<int>, vector<vector<int>>, cmp> heap;
	sort(jobs.begin(), jobs.end(), compare);

	for (i = 0; i < size; i++) {
		answer -= jobs[i][0];
	}

	time = jobs[0][0] + jobs[0][1];
	answer += time;
	for (i = 1; i < size; i++) {
		if (jobs[i][0] > time) {
			do {
				if (heap.empty())break;
				vector<int> tmp = heap.top(); heap.pop();
				time += tmp[1];
				answer += time;
			} while (jobs[i][0] > time);
			if (jobs[i][0] > time) {
				time = jobs[i][0] + jobs[i][1]; answer += time;
			}
			else {
				heap.push(jobs[i]);
			}
		}
		else {
			heap.push(jobs[i]);
		}
	}
	while (!heap.empty()) {
		vector<int> tmp = heap.top(); heap.pop();
		time += tmp[1];
		answer += time;
	}
	return answer / size;
}