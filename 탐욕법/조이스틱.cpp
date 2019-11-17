#include <string>
#include <vector>
#include <algorithm>

using namespace std;

int solution(string name) {
	int answer = 0;
	int cnt = 0, maxA=0;
    int size = name.size();
	vector<int> indexs;
	for (int i = 0; i < size; i++) {
		if (name[i] != 'A') {
			if (i != 0)indexs.push_back(i);
            maxA=max(maxA,cnt);cnt=0;
			answer += min(name[i] - 'A', 26 - (name[i] - 'A'));
		}else{
            cnt++;
        }
	}
	
    if(indexs.size()==0){
        return answer;
    }else if(indexs.size()==1){
        answer+=min(indexs[0],size-indexs[0]);
    }else{
        if(maxA==0){
            answer+=size-1;
            return answer;
        }
        int left,right;
        for(int i=0;i<indexs.size()-1;i++){
            left=indexs[i];right=indexs[i+1];
            if(right-left-1==maxA) break;
        }
        int tmp=2*min(size-right,left)+max(size-right,left);
        answer+=min(tmp,min(indexs.back(),size-indexs.front()));
    }
    return answer;
}