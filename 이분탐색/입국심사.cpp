#include <string>
#include <vector>
#include <algorithm>

using namespace std;

long long solution(int n, vector<int> times) {
    long long answer = 28;
    long long min=0,max=n;
    long long num=0;
    sort(times.begin(),times.end());
    max*=times[0];
    answer=max;
    while(min!=answer){
        num=0;
        for(int i=0;i<times.size();i++){
            num+=answer/times[i];
        }
        if(num<n){   
            min=answer;
            answer+=(n-num)*times[0];
        }
        else{      
            answer=(min+answer+1)/2;
        }
    }
    return answer;
}