#include "wall.h"
#include <include/glm/glm.hpp>
#include <iostream>

using namespace std;

Wall::Wall(glm::vec2 pos, glm::vec2 pos2)
{
    this->point1 = pos;
    this->point2 = pos2;
}

float area2(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y) {
    return (p2x - p1x) * (p3y - p1y) - (p3x - p1x) * (p2y - p1y);
}

bool between(float x1, float y1, float x2, float y2, float x3, float y3) {
    if (x1 != x2) {
       return (x1 <= x3 && x3 <= x2) || (x1 >= x3 && x3 >= x2);
     }
     else {
       return (y1 <= y3 && y3 <= y2) || (y1 >= y3 && y3 >= y2);
     }
}

bool Wall::isInvalidMove(glm::vec2 start, glm::vec2 end)
{

    float x1 = start.x;
    float y1 = start.y;
    float x2 = end.x;
    float y2 = end.y;

    float x3 = this->point1.x;
    float y3 = this->point1.y;
    float x4 = this->point2.x;
    float y4 = this->point2.y;

    bool condition = x1 == 250 && y1 == 70 && x2 == 50 && x3 == 55;
    if (condition) {
    cout << "x1 " << x1 << std::endl;
    cout << "y1 " << 500 -y1 << endl;
    cout << "x2 " << x2 << endl;
    cout << "y2 " << 500-y2 << endl;
    cout << "x3 " << x3 << endl;
    cout << "y3 " << 500-y3 << endl;
    cout << "x4 " << x4 << endl;
    cout << "y4 " << 500-y4 << endl << endl;
    }
    float a1, a2, a3, a4;
    // deal with special cases

    if ((a1 = area2(x1, y1, x2, y2, x3, y3)) == 0.0f) {
        // check if p3 is between p1 and p2 OR
        // p4 is collinear also AND either between p1 and p2 OR at opposite ends
        if (between(x1, y1, x2, y2, x3, y3)) {
            return true;
        } else {
            if (area2(x1, y1, x2, y2, x4, y4) == 0.0f) {
                return between(x3, y3, x4, y4, x1, y1) || between (x3, y3, x4, y4, x2, y2);
            } else {
                return false;
            }
        }
    } else if ((a2 = area2(x1, y1, x2, y2, x4, y4)) == 0.0) {
        // check if p4 is between p1 and p2 (we already know p3 is not collinear
        return between(x1, y1, x2, y2, x4, y4);
    }

    if ((a3 = area2(x3, y3, x4, y4, x1, y1)) == 0.0) {
        // check if p1 is between p3 and p4 OR
        // p2 is collinear also AND either between p1 and p2 OR at opposite ends
        if (between(x3, y3, x4, y4, x1, y1)) {
            return true;
        } else {
            if (area2(x3, y3, x4, y4, x2, y2) == 0.0) {
                return between(x1, y1, x2, y2, x3, y3) || between (x1, y1, x2, y2, x4, y4);
            } else {
                return false;
            }
        }
    } else if ((a4 = area2(x3, y3, x4, y4, x2, y2)) == 0.0) {
        // check if p2 is between p3 and p4 (we already know p1 is not collinear
        return between(x3, y3, x4, y4, x2, y2);
    } else { //test for regular intersection
        bool condition1 = (a1 > 0.0) ^ (a2 > 0.0);
        bool condition2 = (a3 > 0.0) ^ (a4 > 0.0);
        if (condition) {
            cout << "a1 " << a1 << endl;
            cout << "a2 " << a2 << endl;
            cout << "a3 " << a3 << endl;
            cout << "a4 " << a4 << endl;
            cout << "(a1 > 0.0) ^ (a2 > 0.0)" << condition1 << endl;
            cout << "(a3 > 0.0) ^ (a4 > 0.0)" << condition2 << endl;
            cout << "((a1 > 0.0) ^ (a2 > 0.0)) && ((a3 > 0.0) ^ (a4 > 0.0))" << (((a1 > 0.0) ^ (a2 > 0.0)) && ((a3 > 0.0) ^ (a4 > 0.0))) << endl << endl;
        }

        return ((a1 > 0.0) ^ (a2 > 0.0)) && ((a3 > 0.0) ^ (a4 > 0.0));
    }
}

