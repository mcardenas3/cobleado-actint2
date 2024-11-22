#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
using namespace std;

struct Point {
    double x, y;
    Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};

struct Edge {
    int src, dest, weight;
    Edge(int s, int d, int w) : src(s), dest(d), weight(w) {}
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

class NetworkSolver {
private:
    int N;
    vector<vector<int>> distances;
    vector<vector<int>> capacities;
    vector<Point> centrals;
    vector<int> parent;

    int find(int x) {
        if (parent[x] == x) return x;
        return parent[x] = find(parent[x]);
    }
    
    void unite(int x, int y) {
        parent[find(x)] = find(y);
    }

    double distance(const Point& p1, const Point& p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    bool bfs(vector<vector<int>>& rGraph, vector<vector<int>>& flow,
             int source, int sink, vector<int>& parent) {
        vector<bool> visited(N, false);
        queue<int> q;
        q.push(source);
        visited[source] = true;
        parent[source] = -1;

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v = 0; v < N; v++) {
                if (!visited[v] && rGraph[u][v] > 0) {
                    q.push(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }
        return visited[sink];
    }

    int fordFulkerson(vector<vector<int>>& graph, int source, int sink) {
        vector<vector<int>> rGraph = graph;
        vector<vector<int>> flow(N, vector<int>(N, 0));
        int max_flow = 0;

        while (true) {
            vector<int> parent(N, -1);
            if (!bfs(rGraph, flow, source, sink, parent))
                break;

            int path_flow = INT_MAX;
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                path_flow = min(path_flow, rGraph[u][v]);
            }

            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                rGraph[u][v] -= path_flow;
                rGraph[v][u] += path_flow;
                flow[u][v] += path_flow;
                flow[v][u] -= path_flow;
            }

            cout << "\nCamino encontrado con flujo " << path_flow << ":\n";
            for (int v = sink; v != source; v = parent[v]) {
                cout << char('A' + parent[v]) << " -> ";
            }
            cout << char('A' + sink) << endl;

            max_flow = path_flow;
        }

        return max_flow;
    }

public:
    NetworkSolver(int n) : N(n) {
        distances = vector<vector<int>>(N, vector<int>(N));
        capacities = vector<vector<int>>(N, vector<int>(N));
        parent = vector<int>(N);
        for(int i = 0; i < N; i++) parent[i] = i;
    }

    bool readFromFile(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Error al abrir el archivo" << endl;
            return false;
        }

        file >> N;
        
        distances = vector<vector<int>>(N, vector<int>(N));
        capacities = vector<vector<int>>(N, vector<int>(N));
        parent = vector<int>(N);
        for(int i = 0; i < N; i++) parent[i] = i;

        for(int i = 0; i < N; i++) {
            for(int j = 0; j < N; j++) {
                file >> distances[i][j];
            }
        }

        for(int i = 0; i < N; i++) {
            for(int j = 0; j < N; j++) {
                file >> capacities[i][j];
            }
        }

        for(int i = 0; i < N; i++) {
            char c1, c2, c3;
            int x, y;
            file >> c1 >> x >> c2 >> y >> c3;
            centrals.push_back(Point(x, y));
        }

        file.close();
        return true;
    }

    void findMST() {
        vector<Edge> edges;
        for(int i = 0; i < N; i++) {
            for(int j = i + 1; j < N; j++) {
                if(distances[i][j] > 0) {
                    edges.push_back(Edge(i, j, distances[i][j]));
                }
            }
        }
        sort(edges.begin(), edges.end());
        
        cout << "1. Cableado óptimo:\n";
        for(const Edge& e : edges) {
            if(find(e.src) != find(e.dest)) {
                unite(e.src, e.dest);
                cout << "(" << char('A' + e.src) << "," 
                     << char('A' + e.dest) << ")\n";
            }
        }
    }

    void solveTSP() {
        vector<bool> visited(N, false);
        vector<int> path;
        path.push_back(0);
        visited[0] = true;

        cout << "\n2. Ruta de reparto:\n";
        for(int i = 1; i < N; i++) {
            int last = path.back();
            int next = -1;
            int minDist = numeric_limits<int>::max();

            for(int j = 0; j < N; j++) {
                if(!visited[j] && distances[last][j] < minDist) {
                    minDist = distances[last][j];
                    next = j;
                }
            }

            path.push_back(next);
            visited[next] = true;
        }

        for(int i = 0; i < path.size(); i++) {
            cout << char('A' + path[i]);
            if(i < path.size() - 1) cout << "->";
        }
        cout << "->A\n";
    }

    void calculateMaxFlow() {
        cout << "\n3. Flujo máximo (Ford-Fulkerson):\n";
        cout << "Matriz de capacidades:\n";
        for(int i = 0; i < N; i++) {
            for(int j = 0; j < N; j++) {
                cout << capacities[i][j] << " ";
            }
            cout << endl;
        }

        int maxFlow = fordFulkerson(capacities, 0, N-1);
        cout << "\nFlujo máximo del nodo A al nodo D: " << maxFlow << endl;
    }

    void generateVoronoiRegions() {
        cout << "\n4. Regiones de Voronoi:\n";
        const double GRID_SIZE = 100;
        const int POINTS = 12;
        
        for(size_t i = 0; i < centrals.size(); i++) {
            vector<Point> region;
            
            for(int p = 0; p < POINTS; p++) {
                double angle = 2.0 * M_PI * p / POINTS;
                Point point(
                    centrals[i].x + GRID_SIZE * cos(angle),
                    centrals[i].y + GRID_SIZE * sin(angle)
                );
                
                bool isClosest = true;
                for(size_t j = 0; j < centrals.size(); j++) {
                    if(i != j && distance(point, centrals[j]) < distance(point, centrals[i])) {
                        isClosest = false;
                        break;
                    }
                }
                
                if(isClosest) {
                    region.push_back(point);
                }
            }
            
            cout << "Polígono " << (i+1) << ": ";
            for(const Point& p : region) {
                cout << "(" << p.x << "," << p.y << ") ";
            }
            cout << endl;
        }
    }
};

int main() {
    NetworkSolver solver(0);
    
    if(!solver.readFromFile("input.txt")) {
        return 1;
    }

    solver.findMST();
    solver.solveTSP();
    solver.calculateMaxFlow();
    solver.generateVoronoiRegions();

    return 0;
}