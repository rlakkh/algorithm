#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int solution(vector<int> weight) {
    int answer = 0;
    int size=weight.size();
    sort(weight.begin(),weight.end());
    long long sum=0;
    if(weight[0]>1) return weight[0]-1;
    for(int i=0;i<size;i++){
        if(weight[i]>sum+1){
            break;
        }
        sum+=weight[i];
    }
    answer=sum+1;
    return answer;
}