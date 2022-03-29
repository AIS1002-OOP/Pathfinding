
#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "pathfinding/Coordinate.hpp"

#include <sstream>

TEST_CASE("Print coordinate") {

    Coordinate c{1, 1};

    std::stringstream ss;
    ss << c;

    REQUIRE(ss.str() == std::string("[" + std::to_string(c.x) + ", " + std::to_string(c.y) + "]"));
}
