#ifndef WALL_H
#define WALL_H

#include <include/glm/glm.hpp>

class Wall
{
public:
    Wall(glm::vec2 pos, glm::vec2 pos2);

    glm::vec2 point1; //first endpoint
    glm::vec2 point2; //second endpoint
    bool isInvalidMove(glm::vec2 start, glm::vec2 end); // returns true if move intersects wall, false if it does not
};

#endif // WALL_H
