
#ifndef PATHFINDING_CLOSESTHEURISTIC_HPP
#define PATHFINDING_CLOSESTHEURISTIC_HPP

#include "pathfinding/Heuristic.hpp"

#include <cmath>

class ClosestHeuristic : public Heuristic {

public:
    float getCost(TileBasedMap *map, const Coordinate &start, const Coordinate &target) override {
        auto dx = static_cast<float>(target.x - start.x);
        auto dy = static_cast<float>(target.y - start.y);

        float result = std::sqrt((dx * dx) + (dy * dy));
        return result;
    }
};

#endif//PATHFINDING_CLOSESTHEURISTIC_HPP
