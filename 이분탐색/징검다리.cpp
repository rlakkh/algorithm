#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int solution(int distance, vector<int> rocks, int n) {
    int answer = distance;
    int min,max;
    min=1;max=distance+1;
    sort(rocks.begin(),rocks.end());
    
    while(min!=answer){
        int cnt=0, pos=0;
        for(int i=0;i<rocks.size();i++){
            if(answer>rocks[i]-pos){
                cnt++;
            }else{
                pos=rocks[i];
            }
        }
        
        if(cnt<=n){
            min=answer;
        }else{
            max=answer;
        }
        answer=(min+max)/2;
    }
    return answer;
}