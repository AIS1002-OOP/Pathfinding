
#include "GameMap.hpp"

#include "pathfinding/astar/AStar.hpp"
#include "pathfinding/Coordinate.hpp"
#include "pathfinding/TileBasedMap.hpp"
#include "pathfinding/heuristics/ClosestHeuristic.hpp"

#include <iostream>
#include <vector>

void printPath(std::vector<std::string> data, const Path &p) {

    auto &start = p.start();
    auto &target = p.target();
    data[start.y][start.x] = 'S';
    data[target.y][target.x] = 'T';

    size_t length = p.length();
    std::cout << "Path length=" << length << "\n\n";
    for (int i = 1; i < length - 1; i++) {
        Coordinate c = p[i];
        data[c.y][c.x] = 'x';
    }

    for (auto &line: data) {
        std::cout << line << "\n";
    }
}

int main() {

    std::vector<std::string> data{"10011111",
                                  "11011111",
                                  "01111111",
                                  "11111110"};

    std::unique_ptr<TileBasedMap> map = std::make_unique<GameMap>(data);

    Coordinate sx{0, 0};
    Coordinate tx{3, 0};

    AStar a(std::move(map), std::make_unique<ClosestHeuristic>());
    auto path = a.findPath(sx, tx);

    if (path) {
        printPath(data, *path);
    } else {
        std::cerr << "Unable to compute path between " << sx << " and " << tx << std::endl;
    }
}
