#include <string>
#include <vector>
#include <queue>

using namespace std;

int solution(int bridge_length, int weight, vector<int> truck_weights) {
	int answer = 0;
	int wsum = 0;
	queue<int> q;
	for (int i = 0; i < truck_weights.size(); i++) {
		if (q.empty()) {
			wsum = truck_weights[i];
			q.push(truck_weights[i]);
			answer++;
		}
		else {
			while (1) {
				if (q.size() == bridge_length) {
					wsum -= q.front(); q.pop();
				}
				else if (wsum + truck_weights[i] <= weight) {
					wsum += truck_weights[i];
					q.push(truck_weights[i]);
					answer++;
					break;
				}
				else {
					q.push(0);
					answer++;
				}
			}
		}
	}
	answer += bridge_length;
	return answer;
}