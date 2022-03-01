#pragma once
#include "scitos_common/vec2.hpp"
#include <vector>

class Line {

    public:
        void setStart( const Vec2<T> &s) {start_ = s; findFunction ();}
        void setEnd( const Vec2<T> &e) {end_ = e; findFunction();}
        float getConfidance(){return confidance_;}
        void setConfidance(float conf){confidance_ = conf;}
        Line(const Vec2<T>, const Vec2<T> ));
        float slope () {return slope_;}
        float yIntersept () {return yIntersept_}
        std::vector<Vec2<float>> start () {return start_;};
        std::vector<Vec2<float>> end() {return end_;};
        bool isParallel (Line);
        bool connect (Line);
        bool closeToLine();
    private:
        std::vector<Vec2<float>> start_;
        std::vector<Vec2<float>> end_;
        float confidance_ = 0;
        float slope_, yIntersept_;
        void findFunction () {
            slope_ = ( end_.y - start_.y ) / ( end_.x - start_.x );
            yIntersept_ = start_.y - slope_ * start_.x;
        }
        
}
Line::Line (const Vec2<T> &s, const Vec2<T> &e ) {
    start_ = s;  end_ = e; Line::findFunction();
}

bool Line::isParallel (Line line){
    if (slope_ == 0 && line.slope == 0)
    {
        return true;
    }
    if else (slope_ == 0){
        if (abs(line.slope) = 0.05 ){
            return true;
        }
        return false;
    }
     if else (line.slope == 0){
        if (abs(slope_) = 0.05 ){
            return true;
        }
        return false;
    }
    if else ((line.slope / slope_) > 0.95 && < 1.05 ){
        return true;
    }
    return false;
}

bool Line::connect (Line line){
    if (closeToLine(line.start)){
        // TODO find new start
        line.setStart()
        return true;
    }
    if else (closeToLine(line.end)){
        // TODO find new end

        line.setEnd()
        return true;
    }
    return false;
}

bool Line::closeToLine (const Vec2<T> &s) {
    float parameter = 5;
    float d1 = abs(slope_ * s.x + s.y + yIntersept_) / sqrt( slope_^2 + 1 );
    float d2 = sqrt((s.x + start_.x)^2+(s.y + start_.y)^2);
    float d3 = sqrt((s.x + end_.x)^2+(s.y + end_.y)^2);
    float d4 = sqrt((start_.x + end_.x)^2+(start_.y + end_.y)^2);

    // TODO if point is in radius of line function and 
    // not too far from start or end
    if (d1 <= parameter && ())
    { 
        return true;
    }

    return false;
}