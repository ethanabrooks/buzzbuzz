#ifndef LIGHT_H
#define LIGHT_H

#include <QList>
#include <include/glm/glm.hpp>
#include <QColor>

class Light
{
public:
    Light();

    int radius;

    QList<glm::vec2> trail;
    QColor trailColor;
    void moveRandomly(); // just randomly moves Light (definiton is in board.cpp)

    void setInitialPosition(float posx, float posy); // use this method to define initial positions for your Lights
    bool moveTo(float newPosx, float newPosy); // use this method when moving the Lights through the Player class (definition in board.cpp)

    glm::vec2 getPosition();
    void returnToPreviousPosition();

private:
    glm::vec2 position;
};

#endif // LIGHT_H
