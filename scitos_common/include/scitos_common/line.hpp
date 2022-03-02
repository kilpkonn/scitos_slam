#pragma once
#pragma once
#include "scitos_common/vec2.hpp"
#include <vector>
#include <cstdlib>

class Line {

public:
    Line(const std::vector<Vec2<float>>& s, const std::vector<Vec2<float>>& e);
    void setStart(const std::vector<Vec2<float>>& s);
    void setEnd(const std::vector<Vec2<float>>& e);
    float getConfidance();
    void setConfidance(float conf);
    float slope();
    float yIntersept();
    std::vector<Vec2<float>> start();
    std::vector<Vec2<float>> end();
    bool isParallel(Line);
    bool connect(Line);
    bool closeToLine(const std::vector<Vec2<float>>& s);
    bool merge(Line);
    float twoPointDistance(const std::vector<Vec2<float>>& s, const std::vector<Vec2<float>>& e);
    Vec2<T> newPoint(const std::vector<Vec2<float>>& s);
private:
    std::vector<Vec2<float>> start_;
    std::vector<Vec2<float>> end_;
    float confidance_ = 0;
    float slope_;
    float yIntersept_;
    void findFunction();

};


Line::Line(const std::vector<Vec2<float>>& s, const std::vector<Vec2<float>>& e)
{
    start_ = s;  end_ = e; Line::findFunction();
}

void Line::setStart(const std::vector<Vec2<float>>& s) { start_ = s; findFunction(); }

void Line::setEnd(const std::vector<Vec2<float>>& e) { end_ = e; findFunction(); }

float Line::getConfidance() { return confidance_; }

void Line::setConfidance(float conf) { confidance_ = conf; }

float Line::slope() { return slope_; }

float Line::yIntersept() { return yIntersept_; }

std::vector<Vec2<float>> Line::start() { return start_; }

std::vector<Vec2<float>> Line::end() { return end_; }

void Line::findFunction() {

    slope_ = (end_.y - start_.y) / (end_.x - start_.x);
    yIntersept_ = start_.y - slope_ * start_.x;
}

bool Line::isParallel(Line line) { // if lines are parallel and close
    float parameter = 5;
    if (yIntersept_-line.yIntersept() <= parameter) // are line interseptor values close 
    {
        if (slope_ == 0 && (line.slope() == 0))
        {
            return true;
        }
        else if (slope_ == 0) {
            if (abs(line.slope()) == 0.05) {
                return true;
            }
            return false;
        }
        else if (line.slope() == 0) {
            if (abs(slope_) == 0.05) {
                return true;
            }
            return false;
        }
        else if ((line.slope() / slope_) > 0.95 && (line.slope() / slope_) < 1.05) {
            return true;
        }
    }
    return false;
}

bool Line::connect(Line line) {

    if (closeToLine(line.start())) {

        line.setStart(newPoint(line.start()));
            return true;
    }
    else if (closeToLine(line.end())) {

        line.setEnd(newPoint(line.end()));
            return true;
    }
    return false;
}

std::vector<Vec2<float>> Line::newPoint(const std::vector<Vec2<float>>& s) {
    // TODO find new point

    std::vector<Vec2<float>> point;
    return point;
}

bool Line::merge(Line line) { // first to merge all lines and then use connect to modify the points

    // TODO merge two lines
    if (closeToLine(line) && isParallel(line))
    {
        /* should merge

                this should take confidance into consideration?
                maybe confidance1 * confidance2 = newConfidance?

                then take two most far points to make new line
                what to do with old lines? should there be a destructor in the class?

        */
        return true;
    }

    return false;
}

bool Line::closeToLine(const std::vector<Vec2<float>>& s) {

    float parameter = 5; // parameter to regulate how close is close :D
    float d1 = abs(slope_ * s.x + s.y + yIntersept_) / sqrt(pow(slope_, 2) + 1); // point distance to line
    float d2 = twoPointDistance(start_, s); // start distance to point
    float d3 = twoPointDistance(s, end_); // end distance to point
    float d4 = twoPointDistance(start_, end_); // line lenght
    float maxDist = d4 + parameter;

    if (d1 <= parameter && (d3 <= maxDist || d2 <= maxDist))
    {
        return true;
    }

    return false;
}

float Line::twoPointDistance(const std::vector<Vec2<float>>& s, const std::vector<Vec2<float>>& e) {

    float d = sqrt((s.x + e.x) ^ 2 + (s.y + e.y) ^ 2);
    return d;

}
