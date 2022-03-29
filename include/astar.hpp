
#ifndef DJIKSTRA_ASTAR_HPP
#define DJIKSTRA_ASTAR_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include "path.hpp"
#include "tilebasedmap.hpp"

class AStar {

private:
    struct Node {

        int x;
        /** The y coordinate of the node */
        int y;
        /** The path cost for this node */
        float cost = 0;
        /** The parent of this node, how we reached it in the search */
        Node *parent = nullptr;
        /** The heuristic cost of this node */
        float heuristic = 0;
        /** The search depth of this node */
        int depth = 0;

        Node(int x, int y) : x(x), y(y) {}

        int setParent(Node *p) {
            depth = p->depth + 1;
            this->parent = p;

            return depth;
        }

        int compareTo(const Node &other) const {
            float f = heuristic + cost;
            float of = other.heuristic + other.cost;

            if (f < of) {
                return -1;
            } else if (f > of) {
                return 1;
            } else {
                return 0;
            }
        }
    };

    class SortedList {

    private:
        std::vector<Node *> list;

    public:
        Node *first() { return list.front(); }

        void clear() { list.clear(); }

        void add(Node *n) {
            list.push_back(n);
            std::sort(list.begin(), list.end(), [](const Node *n1, const Node *n2) {
                return n1->compareTo(*n2) < 0;
            });
        }

        void remove(const Node *n) {
            list.erase(std::remove(list.begin(), list.end(), n), list.end());
        }

        size_t size() const { return list.size(); }

        bool contains(const Node *n1) const {
            return std::find(list.begin(), list.end(), n1) != std::end(list);
        }
    };

    std::vector<Node *> closed;
    SortedList open;

    TileBasedMap map;
    int maxSearchDistance;

    std::vector<std::vector<Node>> nodes;
    bool allowDiagMovement;

public:
    AStar(TileBasedMap map, int maxSearchDistance,
          bool allowDiagMovement = true)
        : map(std::move(map)), maxSearchDistance(maxSearchDistance),
          allowDiagMovement(allowDiagMovement) {

        for (int x = 0; x < this->map.getWidthInTiles(); x++) {
            nodes.emplace_back();
            for (int y = 0; y < this->map.getHeightInTiles(); y++) {
                nodes[x].emplace_back(x, y);
            }
        }
    }

    bool inClosedList(const Node *n1) {
        return std::find(closed.begin(), closed.end(), n1) != std::end(closed);
    }

    float getHeuristicCost(int x, int y, int tx, int ty) {
        auto dx = static_cast<float>(tx - x);
        auto dy = static_cast<float>(ty - y);

        float result = std::sqrt((dx * dx) + (dy * dy));

        return result;
    }

    bool isValidLocation(int sx, int sy, int x, int y) {
        bool invalid = (x < 0) || (y < 0) || (x >= map.getWidthInTiles()) ||
                       (y >= map.getHeightInTiles());

        if ((!invalid) && ((sx != x) || (sy != y))) {
            invalid = map.blocked(x, y);
        }

        return !invalid;
    }

    float getMovementCost(int sx, int sy, int tx, int ty) {
        return map.getCost(sx, sy, tx, ty);
    }

    std::optional<Path> findPath(int sx, int sy, int tx, int ty) {
        // easy first check, if the destination is blocked, we can't get there
        if (map.blocked(tx, ty)) {
            return std::nullopt;
        }

        // initial state for A*. The closed group is empty. Only the starting
        // tile is in the open list and it's cost is zero, i.e. we're already there
        nodes[sx][sy].cost = 0;
        nodes[sx][sy].depth = 0;
        closed.clear();
        open.clear();
        open.add(&nodes[sx][sy]);

        nodes[tx][ty].parent = nullptr;

        // while we haven't found the goal and haven't exceeded our max search depth
        int maxDepth = 0;
        while ((maxDepth < maxSearchDistance) && (open.size() != 0)) {
            // pull out the first node in our open list, this is determined to
            // be the most likely to be the next step based on our heuristic
            Node *current = open.first();
            if (current == &nodes[tx][ty]) {
                break;
            }

            open.remove(current);
            closed.push_back(current);

            // search through all the neighbours of the current node evaluating
            // them as next steps
            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {
                    // not a neighbour, its the current tile
                    if ((x == 0) && (y == 0)) {
                        continue;
                    }

                    // if we're not allowing diaganol movement then only
                    // one of x or y can be set
                    if (!allowDiagMovement) {
                        if ((x != 0) && (y != 0)) {
                            continue;
                        }
                    }

                    // determine the location of the neighbour and evaluate it
                    int xp = x + current->x;
                    int yp = y + current->y;

                    if (isValidLocation(sx, sy, xp, yp)) {
                        // the cost to get to this node is cost the current plus the
                        // movement cost to reach this node. Note that the heursitic value
                        // is only used in the sorted open list
                        float nextStepCost =
                                current->cost + getMovementCost(current->x, current->y, xp, yp);
                        Node *neighbour = &nodes[xp][yp];
                        //                        map.pathFinderVisited(xp, yp);

                        // if the new cost we've determined for this node is lower than
                        // it has been previously makes sure the node hasn't been discarded.
                        // We've determined that there might have been a better path to get
                        // to this node so it needs to be re-evaluated
                        if (nextStepCost < neighbour->cost) {
                            if (open.contains(neighbour)) {
                                open.remove(neighbour);
                            }
                            if (inClosedList(neighbour)) {
                                closed.erase(
                                        std::remove(closed.begin(), closed.end(), neighbour),
                                        closed.end());
                            }
                        }

                        // if the node hasn't already been processed and discarded then
                        // reset it's cost to our current cost and add it as a next possible
                        // step (i.e. to the open list)
                        if (!open.contains(neighbour) && !(inClosedList(neighbour))) {
                            neighbour->cost = nextStepCost;
                            neighbour->heuristic = getHeuristicCost(xp, yp, tx, ty);
                            maxDepth = std::max(maxDepth, neighbour->setParent(current));
                            open.add(neighbour);
                        }
                    }
                }
            }
        }

        // since we've got an empty open list or we've run out of search
        // there was no path. Just return null
        if (nodes[tx][ty].parent == nullptr) {
            return std::nullopt;
        }

        // At this point we've definitely found a path so we can uses the parent
        // references of the nodes to find out way from the target location back
        // to the start recording the nodes on the way.
        Path path;
        Node *target = &nodes[tx][ty];
        while (target != &nodes[sx][sy]) {
            path.prependStep(target->x, target->y);
            target = target->parent;
        }
        path.prependStep(sx, sy);

        // thats it, we have our path
        return path;
    }
};

#endif// DJIKSTRA_ASTAR_HPP
