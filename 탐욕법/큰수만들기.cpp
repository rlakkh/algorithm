#include <string>
#include <vector>

using namespace std;

string solution(string number, int k) {
    for(int i=1;k>0;i++){
        if(i>=number.size()){
            i=0;
            while(k--)number.pop_back();
            break;
        }
        if(number[i]>number[i-1]){
            number.erase(number.begin()+i-1);
            i=0;k--;
        }
    }
    return number;
}