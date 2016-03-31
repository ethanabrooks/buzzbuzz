#ifndef EXAMPLEPLAYER_H
#define EXAMPLEPLAYER_H

#include "player.h"

class MyPlayer : public Player
{
public:
    MyPlayer();

    // This method will be called before every step (i.e. before mosquitoes are moved).
    // board[x][y] tells you the number of mosquitoes at coordinate (x, y)
    void updateLights(QVector<QVector<int> >* board);

    // This method will only be called once, for the initial placement of the lights
    void initializeLights(QVector<QVector<int> >* board);

    // This method will only be called once per game, for the initial placement of the frog
    glm::vec2 initializeFrog(QVector<QVector<int> >* board);
};

#endif // EXAMPLEPLAYER_H
