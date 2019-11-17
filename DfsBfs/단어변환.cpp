#include <string>
#include <vector>
#include <queue>

using namespace std;

vector<string> wordset;
string goal;
int *used;
int size, ans;

int countDiff(string curr, string next){
    int count=0;
    for(int i=0;i<curr.size();i++){
        count+=(curr[i]!=next[i]);
    }
    return count;
}

void dfs(int v, int step){
    if(wordset[v].compare(goal)==0){
        if(step<ans)ans=step;
        return;
    }
    used[v]=1;
    for(int i=0;i<size;i++){
        if(!used[i]&&countDiff(wordset[v],wordset[i])==1){
            dfs(i,step+1);
        }
    }
    used[v]=0;
}

int solution(string begin, string target, vector<string> words) {
    ans=51;
    int i;
    size = words.size();
    wordset=words;goal=target;
    for(i=0;i<size;i++){
        if(countDiff(begin,words[i])==1){
            int init[51]={0,};
            used=init;
            dfs(i,1);
        }
    }
    if(ans==51) return 0;
    return ans;
}