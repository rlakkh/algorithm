#include <string>
#include <vector>

using namespace std;

int solution(int n, vector<int> lost, vector<int> reserve) {
    int answer = 0;
    int count[31]={0,};
    for(int i=1;i<=n;i++){
        count[i]=1;
    }
    for(int i=0;i<reserve.size();i++){
        count[reserve[i]]++;
    }
    for(int i=0;i<lost.size();i++){
        count[lost[i]]--;
    }
    for(int i=1;i<=n;i++){
        if(count[i]==0){
            if(i==1){
                if(count[i+1]==2){
                    count[i+1]--;
                    count[i]++;
                }
            }else if(i==n){
                if(count[i-1]==2){
                    count[i-1]--;
                    count[i]++;
                }
            }else{
                if(count[i-1]==2){
                    count[i-1]--;
                    count[i]++;
                }else if(count[i+1]==2){
                    count[i+1]--;
                    count[i]++;
                }
            }
        }
    }
    for(int i=1;i<=n;i++){
        if(count[i]!=0){
            answer++;
        }
    }
    return answer;
}