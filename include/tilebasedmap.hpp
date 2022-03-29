
#ifndef DJIKSTRA_TILEBASEDMAP_HPP
#define DJIKSTRA_TILEBASEDMAP_HPP

#include <utility>
#include <vector>
#include <string>

class TileBasedMap {

public:

    TileBasedMap(const std::vector<std::string> &map) : map(map) {}

    char get(int x, int y) {
        return map[y][x];
    }

    int getWidthInTiles() {
        return map[0].size();
    }

    int getHeightInTiles() {
        return map.size();
    }

    bool blocked(int x, int y) {
        char c = get(x, y);
        bool blocked = (c == '0');
        return blocked;
    }

    float getCost(int sx, int sy, int tx, int ty) {
        return 1;
    }

private:
    std::vector<std::string> map;

};

#endif //DJIKSTRA_TILEBASEDMAP_HPP
