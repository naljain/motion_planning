#include <cmath>
#include <iostream>
#include <unordered_map>

// using namespace std;

namespace planner {

struct Cell {
    int x = 0;
    int y = 0;

    Cell (int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const Cell& other_cell) const {
        return other_cell.x == x && other_cell.y == y;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Cell& c){
        os << '(' << c.x << ',' << c.y << ')';
        return os;
    }
};

inline double euclidean_distance(const Cell& a, const Cell& b) {
    double dist = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    return dist;
}

inline double manhattan_distance(const Cell& a, const Cell& b) {
    double dist = abs(a.x - b.x) + abs(a.y - b.y);
    return dist;
}
}

namespace std {
    template<>
    struct hash<planner::Cell> {
        size_t operator()(const planner::Cell& c) const {
            size_t h1 = std::hash<int>{}(c.x);
            size_t h2 = std::hash<int>{}(c.y);
            return h1 ^ (h2 << 1);
        }
    };
}