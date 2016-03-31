#ifndef MOSQUITO_H
#define MOSQUITO_H

#include <include/glm/glm.hpp>
#include "math.h"

class Mosquito
{
public:
    Mosquito();

    glm::vec2 position;
    bool isCaught; // flipped if the mosquito is caught by a light
    bool isEaten; // flipped if the mosquito has been eaten by the frog
    glm::vec2 calculateNextMove(glm::vec2 nearestLightPos);
    void move(glm::vec2 nextMove);
};

#endif // MOSQUITO_H
