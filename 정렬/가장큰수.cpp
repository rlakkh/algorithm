#include <string>
#include <vector>
#include <stdio.h>
#include <algorithm>

using namespace std;

bool compare(int a, int b) {
	int n1, n2;
	char num1[10], num2[10];
	sprintf(num1, "%d%d", a, b);
	sprintf(num2, "%d%d", b, a);
	sscanf(num1, "%d", &n1);
	sscanf(num2, "%d", &n2);
	return n1 > n2;
}

string solution(vector<int> numbers) {
	string answer = "";
	sort(numbers.begin(), numbers.end(), compare);

	if (numbers[0] == 0) {
		answer = "0";
	}
	else {
		for (int i = 0; i < numbers.size(); i++) {
			int number = numbers[i];
			answer += to_string(number);
		}
	}

	return answer;
}