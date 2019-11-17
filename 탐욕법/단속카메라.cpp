#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int Max(int a,int b){
    return (a<b)?b:a;
}

int Min(int a,int b){
    return (a<b)?a:b;
}

bool cmp(vector<int> v1, vector<int> v2){
    if(v1[0]<v2[0]){
        return true;
    }else if(v1[0]==v2[0]){
        return v1[0]<v2[1];
    }
    return false;
}

int solution(vector<vector<int>> routes) {
    vector<int> installed(routes.size());
    sort(routes.begin(),routes.end(),cmp);
    int answer = 0, count=0;
    int min=30000,max=-30000;
    do{
        count=0;min=30000;max=-30000;
        for(int i=0;i<routes.size();i++){
            if(!installed[i]){
                if(Max(max,routes[i][0])<=Min(min,routes[i][1])){
                    installed[i]=1;
                    max=Max(max,routes[i][0]);
                    min=Min(min,routes[i][1]);
                    count++;
                }
            }
        }
        if(count) answer++;
    }while(count>0);
    return answer;
}