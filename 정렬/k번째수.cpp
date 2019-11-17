#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int kthInSorted(vector<int> array, int i, int j, int k) {
	vector<int> tmp;
	tmp.resize(j - i + 1);
	for (int index = i - 1; index < j; index++) {
		tmp[index - i + 1] = array[index];
	}
	sort(tmp.begin(), tmp.end());
	return tmp[k - 1];
}

vector<int> solution(vector<int> array, vector<vector<int>> commands) {
	vector<int> answer;
	int begin = 0, end = 0, kth = 0;
	int kthnum = 0;
	for (int i = 0; i < commands.size(); i++) {
		begin = commands[i][0];
		end = commands[i][1];
		kth = commands[i][2];

		kthnum = kthInSorted(array, begin, end, kth);
		answer.push_back(kthnum);
	}
	return answer;
}