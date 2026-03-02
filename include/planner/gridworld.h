#pragma once

#include <stdexcept>
#include <vector>
#include "types.h"

namespace planner {
class GridWorld{
    public:
        GridWorld(int height, int width);

        void set_obstacle(int x, int y);
        void clear_obstacle(int x, int y);

        bool is_obstacle(int x, int y) const;
        bool is_valid(const planner::Cell& c) const;

        std::vector<planner::Cell> get_neighbours(const planner::Cell& c) const;
        double transition_cost(const planner::Cell& c1, const planner::Cell& c2) const;
        
        int width() const;
        int height() const;

    private : 
        int height_;
        int width_;
        std::vector<std::vector<bool>> grid_;
        std::vector<std::vector<bool>> obs_;

        bool in_bounds(int x, int y) const{
            return ((x > 0 && x <= height_) && (y > 0 && y <= width_));
        }
};
}