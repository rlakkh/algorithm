#include<iostream>

using namespace std;

int ans;
int idx;
char equation[51];

int main()
{
	bool minus = false;
	int tmp = 0;
	cin >> equation;

	while (equation[idx] != 0) {
		if (equation[idx] >= '0' && equation[idx] <= '9') {
			tmp = tmp * 10 + equation[idx] - '0';
		}
		else {
			if (!minus) ans += tmp;
			else ans -= tmp;

			if (equation[idx] == '-') minus = true;
			tmp = 0;
		}
		idx++;
	}
	if (!minus) ans += tmp;
	else ans -= tmp;

	cout << ans << "\n";

	return 0;
}