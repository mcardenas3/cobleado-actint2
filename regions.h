#ifndef REGIONS_H
#define REGIONS_H

#include <vector>
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

// Estructura para representar puntos en el plano
struct Point {
    int x, y;
};

// Función para calcular la distancia euclidiana entre dos puntos
double calculateDistance(const Point &a, const Point &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Función para asignar cada colonia a la central más cercana
vector<vector<Point>> assignColoniesToCentrals(const vector<Point> &colonies, const vector<Point> &centrals) {
    vector<vector<Point>> regions(centrals.size());

    for (const auto &colony : colonies) {
        double minDist = numeric_limits<double>::max();
        int nearestCentral = -1;

        for (int i = 0; i < centrals.size(); ++i) {
            double dist = calculateDistance(colony, centrals[i]);
            if (dist < minDist) {
                minDist = dist;
                nearestCentral = i;
            }
        }

        // Asegurarnos de asignar correctamente
        if (nearestCentral != -1) {
            regions[nearestCentral].push_back(colony);
        }
    }

    return regions;
}

// Función para imprimir las regiones en formato de polígonos
void printRegions(const vector<vector<Point>> &regions) {
    cout << "Lista de polígonos (regiones):\n";
    for (int i = 0; i < regions.size(); ++i) {
        cout << "Central " << i + 1 << ":\n";
        for (const auto &point : regions[i]) {
            cout << "(" << point.x << ", " << point.y << ") ";
        }
        cout << "\n";
    }
}

#endif
