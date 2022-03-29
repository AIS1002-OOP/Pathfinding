
#ifndef PATHFINDING_ASTAR_HPP
#define PATHFINDING_ASTAR_HPP

#include "Heuristic.hpp"
#include "Path.hpp"
#include "TileBasedMap.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

class AStar {

private:
    struct Node {

        // The coordinate of the node
        Coordinate xy;

        float cost = 0;
        /** The parent of this node, how we reached it in the search */
        Node *parent = nullptr;
        /** The heuristic cost of this node */
        float heuristic = 0;
        /** The search depth of this node */
        int depth = 0;

        Node(const Coordinate &c) : xy(c) {}

        int setParent(Node *p) {
            depth = p->depth + 1;
            this->parent = p;

            return depth;
        }

        [[nodiscard]] int compareTo(const Node &other) const {
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
        std::vector<Node *> list_;

    public:
        Node *first() { return list_.front(); }

        void clear() { list_.clear(); }

        void add(Node *n) {
            list_.push_back(n);
            std::sort(list_.begin(), list_.end(), [](const Node *n1, const Node *n2) {
                return n1->compareTo(*n2) < 0;
            });
        }

        void remove(const Node *n) {
            list_.erase(std::remove(list_.begin(), list_.end(), n), list_.end());
        }

        [[nodiscard]] size_t size() const { return list_.size(); }

        bool contains(const Node *n1) const {
            return std::find(list_.begin(), list_.end(), n1) != std::end(list_);
        }
    };

    std::vector<Node *> closed;
    SortedList open;

    std::unique_ptr<TileBasedMap> map;
    std::unique_ptr<Heuristic> heuristic;
    int maxSearchDistance = 100;

    std::vector<std::vector<Node>> nodes;
    bool allowDiagMovement = true;

public:
    AStar(std::unique_ptr<TileBasedMap> map, std::unique_ptr<Heuristic> heuristic)
        : map(std::move(map)), heuristic(std::move(heuristic)) {

        for (int x = 0; x < this->map->width(); x++) {
            nodes.emplace_back();
            for (int y = 0; y < this->map->height(); y++) {
                nodes[x].emplace_back(Coordinate{x, y});
            }
        }
    }

    AStar &setMaxSearchDistance(int distance) {
        maxSearchDistance = distance;
        return *this;
    }
    AStar &setAllowDiagMovement(bool allow) {
        allowDiagMovement = allow;
        return *this;
    }

    bool inClosedList(const Node *n1) const {
        return std::find(closed.begin(), closed.end(), n1) != std::end(closed);
    }

    bool isValidLocation(const Coordinate &c1, const Coordinate &c2) const {
        bool invalid = (c2.x < 0) || (c2.y < 0) || (c2.x >= map->width()) ||
                       (c2.y >= map->height());

        if ((!invalid) && ((c1.x != c2.x) || (c1.y != c2.y))) {
            invalid = map->blocked(c2);
        }

        return !invalid;
    }

    [[nodiscard]] float getMovementCost(const Coordinate &s, const Coordinate &t) const {
        return map->getCost(s, t);
    }

    /**
	 * Get the heuristic cost for the given location. This determines in which
	 * order the locations are processed.
	 *
	 * @param s The coordinate of the tile whose cost is being determined
	 * @param t The coordinate of the target location
     *
	 * @return The heuristic cost assigned to the tile
	 */
    float getHeuristicCost(const Coordinate &s, const Coordinate &t) {
        return heuristic->getCost(map.get(), s, t);
    }

    std::optional<Path> findPath(const Coordinate &s, const Coordinate &t) {
        // easy first check, if the destination is blocked, we can't get there
        if (map->blocked(t)) {
            return std::nullopt;
        }

        // initial state for A*. The closed group is empty. Only the starting
        // tile is in the open list and it's cost is zero, i.e. we're already there
        nodes[s.x][s.y].cost = 0;
        nodes[s.x][s.y].depth = 0;
        closed.clear();
        open.clear();
        open.add(&nodes[s.x][s.y]);

        nodes[t.x][t.y].parent = nullptr;

        // while we haven't found the goal and haven't exceeded our max search depth
        int maxDepth = 0;
        while ((maxDepth < maxSearchDistance) && (open.size() != 0)) {
            // pull out the first node in our open list, this is determined to
            // be the most likely to be the next step based on our heuristic
            Node *current = open.first();
            if (current == &nodes[t.x][t.y]) {
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
                    Coordinate p{x + current->xy.x, y + current->xy.y};

                    if (isValidLocation(s, p)) {
                        // the cost to get to this node is cost the current plus the
                        // movement cost to reach this node. Note that the heursitic value
                        // is only used in the sorted open list
                        float nextStepCost =
                                current->cost + getMovementCost(current->xy, p);
                        Node *neighbour = &nodes[p.x][p.y];
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
                                closed.erase(std::remove(closed.begin(), closed.end(), neighbour), closed.end());
                            }
                        }

                        // if the node hasn't already been processed and discarded then
                        // reset it's cost to our current cost and add it as a next possible
                        // step (i.e. to the open list)
                        if (!open.contains(neighbour) && !(inClosedList(neighbour))) {
                            neighbour->cost = nextStepCost;
                            neighbour->heuristic = getHeuristicCost(p, t);
                            maxDepth = std::max(maxDepth, neighbour->setParent(current));
                            open.add(neighbour);
                        }
                    }
                }
            }
        }

        // since we've got an empty open list or we've run out of search
        // there was no path. Just return null
        if (nodes[t.x][t.y].parent == nullptr) {
            return std::nullopt;
        }

        // At this point we've definitely found a path so we can uses the parent
        // references of the nodes to find out way from the target location back
        // to the start recording the nodes on the way.
        Path path;
        Node *target = &nodes[t.x][t.y];
        while (target != &nodes[s.x][s.y]) {
            path.prependStep(target->xy);
            target = target->parent;
        }
        path.prependStep(s);

        // thats it, we have our path
        return path;
    }
};

#endif// PATHFINDING_ASTAR_HPP
