#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <limits>
#include <chrono>
#include <tuple>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;

// Пользовательская хэш-функция для std::pair
struct PairHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1); // Комбинируем хэши
    }
};

using Node = pair<double, double>; // Узел как пара координат
using Edge = pair<Node, double>;  // Ребро как пара (узел, вес)
using Graph = unordered_map<Node, vector<Edge>, PairHash>; // Граф как список смежности

// Функция для вычисления евклидова расстояния между двумя узлами
double euclideanDistance(const Node& a, const Node& b) {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

// Функция для загрузки графа из файла
Graph loadGraphFromFile(const string& filename) {
    Graph graph;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Не удалось открыть файл: " << filename << endl;
        return graph;
    }

    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        string parent, edges;
        getline(iss, parent, ':');

        Node parentNode;
        sscanf(parent.c_str(), "%lf,%lf", &parentNode.first, &parentNode.second);

        while (getline(iss, edges, ';')) {
            Node childNode;
            double weight;
            sscanf(edges.c_str(), "%lf,%lf,%lf", &childNode.first, &childNode.second, &weight);
            graph[parentNode].push_back({childNode, weight});
        }
    }

    file.close();
    return graph;
}

// Глубокий поиск (DFS)
bool dfs(const Graph& graph, const Node& start, const Node& goal, vector<Node>& path, unordered_map<Node, bool, PairHash>& visited) {
    if (start == goal) {
        path.push_back(start);
        return true;
    }

    visited[start] = true;
    path.push_back(start);

    if (graph.find(start) == graph.end()) {
        path.pop_back();
        return false;
    }

    for (const auto& edge : graph.at(start)) {
        const Node& neighbor = edge.first;
        if (!visited[neighbor] && dfs(graph, neighbor, goal, path, visited)) {
            return true;
        }
    }

    path.pop_back();
    return false;
}

// Поиск в ширину (BFS)
bool bfs(const Graph& graph, const Node& start, const Node& goal, vector<Node>& path) {
    unordered_map<Node, Node, PairHash> parent;
    queue<Node> q;
    unordered_map<Node, bool, PairHash> visited;

    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        if (current == goal) {
            Node temp = goal;
            while (temp != start) {
                path.push_back(temp);
                temp = parent[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (graph.find(current) == graph.end()) {
            continue;
        }

        for (const auto& edge : graph.at(current)) {
            const Node& neighbor = edge.first;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    return false;
}

// Алгоритм Дейкстры
bool dijkstra(const Graph& graph, const Node& start, const Node& goal, vector<Node>& path) {
    unordered_map<Node, double, PairHash> dist;
    unordered_map<Node, Node, PairHash> parent;
    auto cmp = [](const pair<Node, double>& a, const pair<Node, double>& b) { return a.second > b.second; };
    priority_queue<pair<Node, double>, vector<pair<Node, double>>, decltype(cmp)> pq(cmp);

    for (const auto& node : graph) {
        dist[node.first] = numeric_limits<double>::infinity();
    }

    dist[start] = 0;
    pq.push({start, 0});

    while (!pq.empty()) {
        Node current = pq.top().first;
        pq.pop();

        if (current == goal) {
            Node temp = goal;
            while (temp != start) {
                path.push_back(temp);
                temp = parent[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (graph.find(current) == graph.end()) {
            continue;
        }

        for (const auto& edge : graph.at(current)) {
            const Node& neighbor = edge.first;
            double weight = edge.second;

            if (dist[current] + weight < dist[neighbor]) {
                dist[neighbor] = dist[current] + weight;
                parent[neighbor] = current;
                pq.push({neighbor, dist[neighbor]});
            }
        }
    }

    return false;
}

// Алгоритм A*
bool aStar(const Graph& graph, const Node& start, const Node& goal, vector<Node>& path) {
    unordered_map<Node, double, PairHash> gScore, fScore;
    unordered_map<Node, Node, PairHash> parent;
    auto cmp = [](const pair<Node, double>& a, const pair<Node, double>& b) { return a.second > b.second; };
    priority_queue<pair<Node, double>, vector<pair<Node, double>>, decltype(cmp)> pq(cmp);

    for (const auto& node : graph) {
        gScore[node.first] = numeric_limits<double>::infinity();
        fScore[node.first] = numeric_limits<double>::infinity();
    }

    gScore[start] = 0;
    fScore[start] = euclideanDistance(start, goal);
    pq.push({start, fScore[start]});

    while (!pq.empty()) {
        Node current = pq.top().first;
        pq.pop();

        if (current == goal) {
            Node temp = goal;
            while (temp != start) {
                path.push_back(temp);
                temp = parent[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (graph.find(current) == graph.end()) {
            continue;
        }

        for (const auto& edge : graph.at(current)) {
            const Node& neighbor = edge.first;
            double weight = edge.second;

            double tentativeGScore = gScore[current] + weight;
            if (tentativeGScore < gScore[neighbor]) {
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + euclideanDistance(neighbor, goal);
                parent[neighbor] = current;
                pq.push({neighbor, fScore[neighbor]});
            }
        }
    }

    return false;
}

// Основная функция для тестирования
int main() {
    Graph graph = loadGraphFromFile("spb_graph.txt");

    if (graph.empty()) {
        cerr << "Граф не загружен." << endl;
        return 1;
    }

    Node start = {30.4141326, 59.9470649}; // Начальная точка
    Node goal = {30.4145466, 59.9470296};  // Конечная точка

    vector<Node> path;

    // Измерение и запуск DFS
    auto start_time = chrono::high_resolution_clock::now();
    unordered_map<Node, bool, PairHash> visited;
    if (dfs(graph, start, goal, path, visited)) {
        auto end_time = chrono::high_resolution_clock::now();
        cout << "DFS Путь: ";
        for (const auto& node : path) {
            cout << "(" << node.first << ", " << node.second << ") ";
        }
        cout << endl;
        auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
        cout << "DFS Время: " << duration.count() << " мкс (" << duration.count() / 1000.0 << " мс)" << endl;
    } else {
        cout << "Путь не найден с помощью DFS" << endl;
    }
    cout << endl;
    // Очистка пути и измерение BFS
    path.clear();
    start_time = chrono::high_resolution_clock::now();
    if (bfs(graph, start, goal, path)) {
        auto end_time = chrono::high_resolution_clock::now();
        cout << "BFS Путь: ";
        for (const auto& node : path) {
            cout << "(" << node.first << ", " << node.second << ") ";
        }
        cout << endl;
        auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
        cout << "BFS Время: " << duration.count() << " мкс (" << duration.count() / 1000.0 << " мс)" << endl;
    } else {
        cout << "Путь не найден с помощью BFS" << endl;
    }
    cout << endl;
    // Очистка пути и измерение Dijkstra
    path.clear();
    start_time = chrono::high_resolution_clock::now();
    if (dijkstra(graph, start, goal, path)) {
        auto end_time = chrono::high_resolution_clock::now();
        cout << "Dijkstra Путь: ";
        for (const auto& node : path) {
            cout << "(" << node.first << ", " << node.second << ") ";
        }
        cout << endl;
        auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
        cout << "Dijkstra Время: " << duration.count() << " мкс (" << duration.count() / 1000.0 << " мс)" << endl;
    } else {
        cout << "Путь не найден с помощью Dijkstra" << endl;
    }
    cout << endl;
    // Очистка пути и измерение A*
    path.clear();
    start_time = chrono::high_resolution_clock::now();
    if (aStar(graph, start, goal, path)) {
        auto end_time = chrono::high_resolution_clock::now();
        cout << "A* Путь: ";
        for (const auto& node : path) {
            cout << "(" << node.first << ", " << node.second << ") ";
        }
        cout << endl;
        auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
        cout << "A* Время: " << duration.count() << " мкс (" << duration.count() / 1000.0 << " мс)" << endl;
    } else {
        cout << "Путь не найден с помощью A*" << endl;
    }

    return 0;
}
