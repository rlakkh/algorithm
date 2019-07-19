#include <string>
#include <vector>

using namespace std;

vector<vector<int>> edge;
int N;
int *visited;
int count;

void dfs(int v){
    visited[v]=count;
    for(int i=0;i<N;i++){
        if(!visited[i]&&edge[v][i]==1){
            dfs(i);
        }
    }
}

int solution(int n, vector<vector<int>> computers){
    int v[200]={0,};
    count=0;N=n;
    visited=v;edge=computers;    
    
    for(int i=0;i<n;i++){
        if(!visited[i]){
            count++;
            dfs(i);
        }
    }
    return count;
}