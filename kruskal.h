#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <vector>
#include <algorithm>

using namespace std;

struct Edge {
    int u, v;
    int weight;
    bool operator<(const Edge &e) const {
        return weight < e.weight;
    }
};

int find(vector<int> &parent, int x) {
    if (parent[x] != x) parent[x] = find(parent, parent[x]);
    return parent[x];
}

void unite(vector<int> &parent, vector<int> &rank, int x, int y) {
    int rootX = find(parent, x);
    int rootY = find(parent, y);
    if (rootX != rootY) {
        if (rank[rootX] > rank[rootY]) parent[rootY] = rootX;
        else if (rank[rootX] < rank[rootY]) parent[rootX] = rootY;
        else {
            parent[rootY] = rootX;
            rank[rootX]++;
        }
    }
}

vector<Edge> kruskal(int n, vector<vector<int>> &graph) {
    vector<Edge> edges, mst;
    vector<int> parent(n), rank(n, 0);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (graph[i][j] != 0)
                edges.push_back({i, j, graph[i][j]});

    sort(edges.begin(), edges.end());
    for (int i = 0; i < n; ++i) parent[i] = i;

    for (auto &e : edges) {
        if (find(parent, e.u) != find(parent, e.v)) {
            mst.push_back(e);
            unite(parent, rank, e.u, e.v);
        }
    }
    return mst;
}

#endif
