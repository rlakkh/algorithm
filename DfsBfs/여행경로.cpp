#include <algorithm>
#include <string>
#include <vector>

using namespace std;

bool first;
int size;
vector<vector<string>> edges;vector<int> used;
vector<string> ans,answer;

bool isSame(string s1,string s2){
    return s1.compare(s2)==0;
}

bool compare(vector<string> v1, vector<string> v2){
    if(v1[1].compare(v2[1])<0){
        return true;
    }else if(v1[1].compare(v2[1])==0){
        return v1[0].compare(v2[0])<0;
    }
    return false;
}

void dfs(int v,int index=1){
    ans[index]=edges[v][1];
    used[v]=1; 
    if(index==size){
        if(!first) return;
        first=false;
        answer=ans;
        return;
    }   
    for(int i=0;i<edges.size();i++){
        if(!used[i]&&isSame(edges[v][1],edges[i][0])){
            dfs(i,index+1);
        }
    }
    used[v]=0;
}

vector<string> solution(vector<vector<string>> tickets) {
    first=true;
    size = tickets.size();
    ans.resize(size+1);
    used.resize(size);
    sort(tickets.begin(),tickets.end(),compare);
    edges=tickets;
    for(int i=0;i<tickets.size();i++){
        if(tickets[i][0].compare("ICN")==0&&!used[i]){
            ans[0]="ICN";
            dfs(i);
        }
    }
    return answer;
}