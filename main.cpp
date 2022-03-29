
#include "astar.hpp"
#include "tilebasedmap.hpp"
#include <iostream>
#include <vector>

int main() {

    std::vector<std::string> data{"10011111",
                                  "01011111",
                                  "01111111",
                                  "11111110"};

    int sx = 0;
    int sy = 0;
    int tx = 3;
    int ty = 0;

    AStar a(data, 10);
    auto p = a.findPath(sx, sy, tx, ty);

    if (p) {

        std::vector<std::string> copy = data;
        copy[sy][sx] = 'S';
        copy[ty][tx] = 'T';

        std::cout << "Path length=" << p->length() << std::endl;
        for (int i = 1; i < p->length() - 1; i++) {
            Step s = p->operator[](i);
            copy[s.getY()][s.getX()] = 'x';
        }

        for (auto &line: copy) {
            std::cout << line << "\n";
        }
    }
}
