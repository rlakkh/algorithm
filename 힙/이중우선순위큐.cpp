#include <string>
#include <vector>
#include <queue>

using namespace std;

typedef struct {
	int value; int index;
}element;

struct maxcmp {
	bool operator()(element e1, element e2) {
		return e1.value < e2.value;
	}
};

struct mincmp {
	bool operator()(element e1, element e2) {
		return e1.value > e2.value;
	}
};

vector<int> solution(vector<string> operations) {
	vector<int> answer(2);
	bool ispop[1000000] = { 0, };
	priority_queue<element, vector<element>, mincmp> minheap;
	priority_queue<element, vector<element>, maxcmp> maxheap;
	for (int i = 0; i < operations.size(); i++) {
		int sign = 1; int num = 0;
		if (operations[i][2] == '-') {
			sign = -1;
		}
		else {
			num = num * 10 + (operations[i][2] - '0');
		}
		for (int j = 3; j < operations[i].size(); j++) {
			num = num * 10 + (operations[i][j] - '0');
		}
		num *= sign;

		if (operations[i][0] == 'I') {
			minheap.push({ num,i }); maxheap.push({ num,i });
		}
		else {
			if (num == -1) {
				if (!minheap.empty()) {
					while (!minheap.empty()) {
						element e = minheap.top(); minheap.pop();
						if (!ispop[e.index]) {
							ispop[e.index] = 1;
							break;
						}
					}
				}
			}
			else {
				if (!maxheap.empty()) {
					while (!maxheap.empty()) {
						element e = maxheap.top(); maxheap.pop();
						if (!ispop[e.index]) {
							ispop[e.index] = 1;
							break;
						}
					}
				}
			}
		}
	}

	if (!maxheap.empty() && !minheap.empty()) {
		while (!minheap.empty()) {
			element e = minheap.top(); minheap.pop();
			if (!ispop[e.index]) {
				answer[1] = e.value;
				break;
			}
		}while (!maxheap.empty()) {
			element e = maxheap.top(); maxheap.pop();
			if (!ispop[e.index]) {
				answer[0] = e.value;
				break;
			}
		}
	}

	return answer;
}