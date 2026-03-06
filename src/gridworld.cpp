#include "planner/gridworld.h"
#include <cmath>

namespace planner {
GridWorld::GridWorld (int height, int width) : 
    width_(width), height_(height), 
    grid_(height, std::vector<bool>(width, false)){

        if (width <= 0 || height <= 0){
            throw std::invalid_argument("Invalid height/width");
        }
    }

void GridWorld::set_obstacle(int x, int y){
    if (!in_bounds(x, y)){
        throw std::invalid_argument("Out of bounds x, y");
    }
    grid_[y][x] = true;
}

void GridWorld::clear_obstacle(int x, int y){
    if (!in_bounds(x, y)){
        throw std::out_of_range("Out of bounds x, y");
    }
    grid_[y][x] = false;
}

bool GridWorld::is_obstacle(int x, int y) const {
    if (!in_bounds(x, y)){
        return true;
    }
    return grid_[y][x];
}

bool GridWorld::is_valid(const Cell&c) const{
    return (in_bounds(c.x, c.y) && !is_obstacle(c.x, c.y));
}

std::vector<Cell> GridWorld::get_neighbours(const Cell& c) const{
    int dir_x[] = {0, 0, 1, -1};
    int dir_y[] = {1, -1, 0, 0};

    std::vector<Cell> neighbours = {};

    for (int i = 0; i < 4; i ++){
        neighbours.push_back(Cell(c.x + dir_x[i], c.y + dir_y[i]));
    }
    return neighbours; 
}

double GridWorld::transition_cost(const Cell& c1, const Cell& c2) const {
    double cost = euclidean_distance(c1, c2);
    return cost;
}

}