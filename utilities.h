#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

void readInput(const string &filename, int &n, vector<vector<int>> &distanceGraph,
               vector<vector<int>> &capacityGraph, vector<pair<int, int>> &locations) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "No se pudo abrir el archivo de entrada.\n";
        exit(EXIT_FAILURE);
    }

    file >> n;
    distanceGraph.resize(n, vector<int>(n));
    capacityGraph.resize(n, vector<int>(n));

    // Leer la matriz de distancias
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            file >> distanceGraph[i][j];

    // Leer la matriz de capacidades
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            file >> capacityGraph[i][j];

    // Leer las ubicaciones
    for (int i = 0; i < n; ++i) {
        int x, y;
        file >> x >> y;
        locations.emplace_back(x, y);
    }

    file.close();
}

#endif
