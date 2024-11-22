#ifndef TSP_H
#define TSP_H

#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

int tsp(int n, vector<vector<int>> &graph, vector<int> &path) {
    vector<int> cities(n);
    for (int i = 0; i < n; ++i) cities[i] = i;
    int minCost = numeric_limits<int>::max();

    do {
        int currentCost = 0;
        for (int i = 0; i < n - 1; ++i)
            currentCost += graph[cities[i]][cities[i + 1]];
        currentCost += graph[cities[n - 1]][cities[0]];

        if (currentCost < minCost) {
            minCost = currentCost;
            path = cities;
        }
    } while (next_permutation(cities.begin() + 1, cities.end()));

    return minCost;
}

#endif
