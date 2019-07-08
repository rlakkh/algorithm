#include <string>
#include <vector>
#include <queue>

using namespace std;

int solution(int stock, vector<int> dates, vector<int> supplies, int k) {
	int answer = 0;
	int index = 0;
	priority_queue<int> heap;
	for (int date = 0; date < k; date++) {
		if (stock == 0) {
			if (dates[index] == date) {
				heap.push(supplies[index]);
				index++;
			}
			stock = heap.top(); answer++;
			heap.pop();
		}
		else {
			if (dates[index] == date) {
				heap.push(supplies[index]);
				index++;
			}
		}
		stock--;
	}
	return answer;
}