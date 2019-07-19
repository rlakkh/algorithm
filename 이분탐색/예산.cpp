#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int solution(vector<int> budgets, int M) {
    int answer = 0;
    int size = budgets.size();
    int s,e,m;
    int i;
    sort(budgets.begin(),budgets.end());
    for(i=0;i<size;i++){
        if(M>(size-i)*budgets[i]){
            M-=budgets[i];
        }else{
            answer=M/(size-i);
            break;
        }
    }
    if(i==size){
        answer=budgets[i-1];
    }
    return answer;
}