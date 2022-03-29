
#ifndef DJIKSTRA_PATH_HPP
#define DJIKSTRA_PATH_HPP

#include <vector>

class Step {

public:

    Step(int x, int y) : x(x), y(y) {}

    int getX() const {
        return x;
    }

    int getY() const {
        return y;
    }

    bool operator==(const Step &other) const {
        return x == other.x && y == other.y;
    }

private:
    int x, y;
};


class Path {

public:
    size_t length() const {
        return steps.size();
    }

    Step operator[](unsigned int index) const {
        return steps[index];
    }

    void prependStep(int x, int y) {
        steps.insert(steps.begin(), Step{x, y});
    }

    void appendStep(int x, int y) {
        steps.emplace_back(Step{x, y});
    }

    bool contains(int x, int y) const {
        return std::find(steps.begin(), steps.end(), Step(x, y)) != steps.end();
    }

private:
    std::vector<Step> steps;
};

#endif //DJIKSTRA_PATH_HPP
