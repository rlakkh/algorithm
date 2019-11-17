#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int solution(vector<int> citations) {
	int answer = 0;
	int size = citations.size();
	sort(citations.begin(), citations.end());
	for (int i = size - 1; i >= 0; i--) {
		if (size - i > citations[i])break;
		answer = size - i;
	}
	return answer;
}