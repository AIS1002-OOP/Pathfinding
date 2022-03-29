
#include <iostream>
#include <vector>
#include "tilebasedmap.hpp"
#include "astar.hpp"

int main() {

    TileBasedMap map = std::vector<std::string>{
            "11111111",
            "11111111",
            "11111111",
            "11111111"
    };

    AStar a(map);
    auto p = a.findPath(0, 0, 2, 0);

    if (p) {
        std::cout << p->length();
    }


}

