
#ifndef PATHFINDING_HEURISTIC_HPP
#define PATHFINDING_HEURISTIC_HPP

#include "Coordinate.hpp"
#include "TileBasedMap.hpp"

class Heuristic {

public:
    virtual float getCost(TileBasedMap *map, const Coordinate &start, const Coordinate &target) = 0;

    virtual ~Heuristic() = default;
};

#endif//PATHFINDING_HEURISTIC_HPP
