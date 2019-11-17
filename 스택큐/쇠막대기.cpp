#include <string>
#include <stack>
#include <vector>

using namespace std;

int solution(string arrangement) {
	int answer = 0;
	stack<char> s;
	s.push(arrangement[0]);
	for (int i = 1; i < arrangement.size(); i++) {
		if (arrangement[i] == '(') {
			s.push(arrangement[i]);
		}
		else {
			s.pop();
			if (arrangement[i - 1] == '(') {
				answer += (int)s.size();
			}
			else {
				answer++;
			}
		}
	}

	return answer;
}