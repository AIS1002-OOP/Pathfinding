
#ifndef DJIKSTRA_PATH_HPP
#define DJIKSTRA_PATH_HPP

#include <ostream>
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

    friend std::ostream &operator<<(std::ostream &os, const Step &s) {
        os << "[" << s.x << "," << s.y << "]";
        return os;
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

private:
    std::vector<Step> steps;
};

#endif//DJIKSTRA_PATH_HPP
