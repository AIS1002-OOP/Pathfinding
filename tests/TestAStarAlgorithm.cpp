

#define CATCH_CONFIG_MAIN
#include "GameMap.hpp"
#include "catch.hpp"

#include "pathfinding/astar/AStar.hpp"
#include "pathfinding/heuristics/ClosestHeuristic.hpp"

TEST_CASE("Test algorithm") {

    std::vector<std::string> data{"10011111",
                                  "11011111",
                                  "01111111",
                                  "11111110"};

    std::unique_ptr<TileBasedMap> map = std::make_unique<GameMap>(data);

    Coordinate sx{0, 0};
    Coordinate tx{3, 0};

    AStar a(std::move(map), std::make_unique<ClosestHeuristic>());

    {
        a.setAllowDiagMovement(true);

        auto path = a.findPath(sx, tx);
        REQUIRE(path);

        CHECK(path->contains({0,0}));
        CHECK(path->contains({1,1}));
        CHECK(path->contains({2,2}));
        CHECK(path->contains({3,1}));
        CHECK(path->contains({3,0}));
    }

    {
        a.setAllowDiagMovement(false);

        auto path = a.findPath(sx, tx);
        REQUIRE(path);

        CHECK(path->contains({0,0}));
        CHECK(path->contains({0,1}));
        CHECK(path->contains({1,1}));
        CHECK(path->contains({1,2}));
        CHECK(path->contains({2,2}));
        CHECK(path->contains({3,2}));
        CHECK(path->contains({3,1}));
        CHECK(path->contains({3,0}));
    }

}
