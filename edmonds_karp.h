#ifndef EDMONDS_KARP_H
#define EDMONDS_KARP_H

#include <vector>
#include <queue>
#include <limits>

using namespace std;

int edmondsKarp(int n, vector<vector<int>> &capacity, int source, int sink) {
    vector<vector<int>> residual = capacity;
    vector<int> parent(n);
    int maxFlow = 0;

    while (true) {
        fill(parent.begin(), parent.end(), -1);
        queue<pair<int, int>> q;
        q.push({source, numeric_limits<int>::max()});

        while (!q.empty() && parent[sink] == -1) {
            int u = q.front().first;
            int flow = q.front().second;
            q.pop();

            for (int v = 0; v < n; ++v) {
                if (parent[v] == -1 && residual[u][v] > 0) {
                    parent[v] = u;
                    int newFlow = min(flow, residual[u][v]);
                    if (v == sink) {
                        flow = newFlow;
                        break;
                    }
                    q.push({v, newFlow});
                }
            }
        }

        if (parent[sink] == -1) break;

        int flow = numeric_limits<int>::max();
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            flow = min(flow, residual[u][v]);
        }

        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= flow;
            residual[v][u] += flow;
        }

        maxFlow += flow;
    }

    return maxFlow;
}

#endif
