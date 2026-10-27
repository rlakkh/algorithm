// 구간 [l,r] 을 경계점 (l-1) -> r 간선으로 보면,
// 필요한 최소 직접 질의 수 = 그래프 (정점 0..n, 간선 (l_i-1, r_i)) 의 랭크
//                         = (n+1) - 연결 요소 수.
//  - 하한: 구간 지시벡터는 f_r - f_{l-1} 로 사상되고, 조합은 합이므로 |S| >= rank.
//  - 달성: 각 연결 요소의 정점을 정렬해 v1<v2<...<vk 이면 [v_j+1, v_{j+1}] 만 질의.
// 교체 쿼리는 간선 삭제/추가 -> 오프라인 동적 연결성 (시간 세그트리 + 롤백 DSU).
#include <bits/stdc++.h>
using namespace std;

static char ibuf[1 << 25];
int ipos, ilen;
inline int readChar() {
    if (ipos == ilen) {
        ilen = (int)fread(ibuf, 1, sizeof(ibuf), stdin);
        ipos = 0;
        if (ilen <= 0) return -1;
    }
    return ibuf[ipos++];
}
inline int readInt() {
    int c = readChar();
    while (c != '-' && (c < '0' || c > '9')) c = readChar();
    int x = 0;
    while (c >= '0' && c <= '9') { x = x * 10 + (c - '0'); c = readChar(); }
    return x;
}

vector<int> par, sz, stk;
int rnk;

inline int findRoot(int x) {
    while (par[x] != x) x = par[x];
    return x;
}
inline void unite(int a, int b) {
    a = findRoot(a); b = findRoot(b);
    if (a == b) return;
    if (sz[a] < sz[b]) swap(a, b);
    par[b] = a; sz[a] += sz[b];
    stk.push_back(b);
    rnk++;
}
inline void rollback(int to) {
    while ((int)stk.size() > to) {
        int b = stk.back(); stk.pop_back();
        int a = par[b];
        sz[a] -= sz[b]; par[b] = b;
        rnk--;
    }
}

int P, Q;
vector<int> eu, ev, cnt, startIdx, items, ans;

void dfs(int node, int lo, int hi) {
    if (lo > Q) return;
    int saved = (int)stk.size();
    for (int i = startIdx[node]; i < startIdx[node + 1]; i++) unite(eu[items[i]], ev[items[i]]);
    if (lo == hi) ans[lo] = rnk;
    else {
        int mid = (lo + hi) >> 1;
        dfs(node * 2, lo, mid);
        dfs(node * 2 + 1, mid + 1, hi);
    }
    rollback(saved);
}

int main() {
    int t = readInt();
    string out;
    out.reserve(1 << 22);
    while (t--) {
        int n = readInt(), m = readInt(), q = readInt();
        Q = q;
        P = 1;
        while (P < q + 1) P <<= 1;

        // 각 간선의 생존 시간 구간 [s, e]
        vector<int> curE(m), curS(m, 0), es, ee;
        eu.clear(); ev.clear();
        eu.reserve(m + q); ev.reserve(m + q); es.reserve(m + q); ee.reserve(m + q);
        for (int i = 0; i < m; i++) {
            int l = readInt(), r = readInt();
            curE[i] = (int)eu.size();
            eu.push_back(l - 1); ev.push_back(r);
        }
        es.assign(m + q, 0); ee.assign(m + q, 0);
        for (int tm = 1; tm <= q; tm++) {
            int p = readInt() - 1, l = readInt(), r = readInt();
            int id = curE[p];
            es[id] = curS[p]; ee[id] = tm - 1;
            curE[p] = (int)eu.size(); curS[p] = tm;
            eu.push_back(l - 1); ev.push_back(r);
        }
        for (int i = 0; i < m; i++) { es[curE[i]] = curS[i]; ee[curE[i]] = q; }
        int E = (int)eu.size();

        // 시간 세그트리에 간선 배치 (CSR)
        cnt.assign(2 * P + 1, 0);
        for (int id = 0; id < E; id++) {
            for (int l = es[id] + P, r = ee[id] + P + 1; l < r; l >>= 1, r >>= 1) {
                if (l & 1) cnt[l++]++;
                if (r & 1) cnt[--r]++;
            }
        }
        startIdx.assign(2 * P + 1, 0);
        for (int i = 0; i < 2 * P; i++) startIdx[i + 1] = startIdx[i] + cnt[i];
        items.assign(startIdx[2 * P], 0);
        for (int i = 0; i < 2 * P; i++) cnt[i] = startIdx[i];
        for (int id = 0; id < E; id++) {
            for (int l = es[id] + P, r = ee[id] + P + 1; l < r; l >>= 1, r >>= 1) {
                if (l & 1) items[cnt[l++]++] = id;
                if (r & 1) items[cnt[--r]++] = id;
            }
        }

        par.resize(n + 1); sz.assign(n + 1, 1);
        for (int i = 0; i <= n; i++) par[i] = i;
        stk.clear(); rnk = 0;
        ans.assign(q + 1, 0);
        dfs(1, 0, P - 1);

        for (int i = 0; i <= q; i++) { out += to_string(ans[i]); out += '\n'; }
    }
    fwrite(out.data(), 1, out.size(), stdout);
    return 0;
}
