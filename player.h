#ifndef PLAYER_H
#define PLAYER_H

#include <include/glm/glm.hpp>
#include <QList>
#include "wall.h"
#include "light.h"

class Player
{

public:
    // Player is an abstract base class that your class MUST inherit from.
    Player();

    QList<Light*> lights; // These are the actual lights in Board (i.e. you can and should move them)
    QList<Wall*> walls; // This is just a copy of the walls in Board (i.e. you cannot move the walls from Player)
    QString playerName;

    // This method will be called before every step (i.e. before mosquitoes are moved).
    // board[x][y] tells you the number of mosquitoes at coordinate (x, y)
    virtual void updateLights(QVector<QVector<int> >* board) = 0;

    // This method will only be called once, for the initial placement of the lights
    virtual void initializeLights(QVector<QVector<int> >* board) = 0;

    // This method will only be called once, for the initial placement of the frog
    virtual glm::vec2 initializeFrog(QVector<QVector<int> >* board) = 0;

    virtual ~Player() {}

};

#endif // PLAYER_H
