#include <string>
#include <vector>

using namespace std;

vector<int> solution(vector<int> progresses, vector<int> speeds) {
	vector<int> answer;
	vector<int> terms;
	for (int i = 0; i < progresses.size(); i++) {
		int term = (100 - progresses[i]) / speeds[i] + ((100 - progresses[i]) % speeds[i] != 0);
		terms.push_back(term);
	}
	int index = 0;
	answer.push_back(1);
	for (int i = 1; i < terms.size(); i++) {
		if (terms[i] < terms[i - 1]) {
			terms[i] = terms[i - 1];
		}
	}for (int i = 1; i < terms.size(); i++) {
		if (terms[i] == terms[i - 1]) {
			answer[index]++;
		}
		else {
			index++;
			answer.push_back(1);
		}
	}
	return answer;
}