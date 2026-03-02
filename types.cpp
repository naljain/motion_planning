#include "types.h"
#include <iostream>
#include <cmath>
#include <unordered_map>

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
    
    std::cout << g_score[start] << "\n";  // 0
    
    return 0;

}

