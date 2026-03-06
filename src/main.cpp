#include "planner/types.h"
#include <iostream>
#include <cmath>
#include <unordered_map>
#include "planner/gridworld.h"

using namespace planner;

int main (){
    Cell a(0, 0);
    Cell b(3, 8);
    std::cout << a << " to " << b << "\n";
    std::cout << "Euclidean: " << euclidean_distance(a, b) << "\n";
    std::cout << "Manhattan: " << manhattan_distance(a, b) << "\n";
    std::unordered_map<Cell, double> g_score;
    
    Cell start(0, 0);
    Cell goal(10, 10);
    
    g_score[start] = 0.0;      // ✅ Works now!
    g_score[goal] = 14.14;
    
    std::cout << "g sccore" << g_score[start] << "\n";  // 0
    

    GridWorld world(10, 10);

    std::cout << "gridworld";

    // Add wall
    for (int y = 0; y < 10; y++) {
        world.set_obstacle(5, y);
    }

    // Check cells
    std::cout << world.is_obstacle(5, 5) << "\n";  // true
    std::cout << world.is_valid(Cell(5, 5)) << "\n";  // false
    std::cout << world.is_valid(Cell(4, 5)) << "\n";  // true

    // Get neighbors
    auto neighbors = world.get_neighbours(Cell(4, 5));
    std::cout << "Neighbors: " << neighbors.size() << "\n";
        
    return 0;

}

