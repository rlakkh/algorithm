#include <string>
#include <vector>

using namespace std;
vector<int> set;
int ans=0;

void count(int index,int target){
    if(index==set.size()){
        if(target==0) ans++;
        return;
    }
    count(index+1,target-set[index]);
    count(index+1,target+set[index]);
}

int solution(vector<int> numbers, int target) {
    ans=0;
    set=numbers;
    count(0,target);    
    return ans;
}