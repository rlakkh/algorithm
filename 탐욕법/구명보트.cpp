#include <string>
#include <vector>

using namespace std;

int solution(vector<int> people, int limit) {
    int answer=0;
    int last=0;
    int count[241]={0,};
    for(int i=0;i<people.size();i++){        
        count[people[i]]++;
    }
    for(int i=40;i<=240;i++){
        while(count[i]--){
            people[last++]=i;
        }
    }
    
    for(int i=0;i<last;){
        if(people[last-1]+people[i]>limit){
            last--;
            answer++;
        }else{
            i++;last--;
            answer++;
        }
    }
    
    return answer;
}