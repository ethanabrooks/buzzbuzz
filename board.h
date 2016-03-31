#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <QList>
#include <QVector>
#include "mosquito.h"
#include "light.h"
#include "wall.h"
#include "frog.h"
#include "player.h"
#include "myplayer.h"

class Board
{
public:
    Board();

    QList<Mosquito*> mosquitoes;
    QList<Light*> lights;
    static QList<Wall*> walls;
    Frog* frog;
    Player* player;
    QVector<QVector<int> > playerBoard; // a 2d array that contains the number of mosquitoes at each position; passed to the Playe

    int numMosquitoes;
    int numLights;
    int numWalls;
    int mosquitoesEaten;
    int mosquitoesCaught;
    static int boardSize; // the Board is a square of boardSize x boardSize
    int captureTarget; // number of mosquitoes to catch in order to win
    int maxRounds; // max number of rounds/steps before game ends
    int currRound; // shows which round we're currently on

    void initialize();
    void step();
    void updateMosquitoesEaten();
    void generateBoardForPlayer();
    static bool checkValidMove(glm::vec2 oldPos, glm::vec2 newPos); // checks if a move is valid (i.e. doesn't go through walls or beyond boundaries)
};


#endif // DRAWABLE_H
