#include "kruskal.h"
#include "tsp.h"
#include "edmonds_karp.h"
#include "utilities.h"
#include "regions.h"
#include <iostream>

int main() {
    int n;
    vector<vector<int>> distanceGraph;
    vector<vector<int>> capacityGraph;
    vector<pair<int, int>> locations;

    string filename = "input.txt";
    readInput(filename, n, distanceGraph, capacityGraph, locations);

    // Convertir las ubicaciones a la estructura Point
    vector<Point> colonies;
    vector<Point> centrals;
    for (const auto &loc : locations) {
        colonies.push_back({loc.first, loc.second});
        centrals.push_back({loc.first, loc.second}); // Supongamos que todas las colonias son posibles centrales
    }

    // MST para el cableado
    vector<Edge> mst = kruskal(n, distanceGraph);
    cout << "Cableado con fibra óptica:\n";
    for (auto &e : mst) cout << "(" << char('A' + e.u) << ", " << char('A' + e.v) << ")\n";

    // Ruta TSP
    vector<int> tspPath;
    int tspCost = tsp(n, distanceGraph, tspPath);
    cout << "Ruta del repartidor:\n";
    for (int i : tspPath) cout << char('A' + i) << " ";
    cout << char('A' + tspPath[0]) << "\n";

    // Flujo máximo
    int maxFlow = edmondsKarp(n, capacityGraph, 0, n - 1);
    cout << "Flujo máximo: " << maxFlow << "\n";

    // Generar y mostrar los polígonos
    vector<vector<Point>> regions = assignColoniesToCentrals(colonies, centrals);
    printRegions(regions);

    return 0;
}
