
#ifndef PATHFINDING_MANHATTANHEURISTIC_HPP
#define PATHFINDING_MANHATTANHEURISTIC_HPP

#include "astar/Heuristic.hpp"

class ManhattanHeuristic : public Heuristic {

public:
    ManhattanHeuristic(int minimumCost) : minimumCost_(minimumCost) {}

    float getCost(TileBasedMap *map, const Coordinate &start, const Coordinate &target) override {
        return minimumCost_ * (std::abs(start.x - target.x) + std::abs(start.y - target.y));
    }

private:
    int minimumCost_;
};

#endif//PATHFINDING_MANHATTANHEURISTIC_HPP
