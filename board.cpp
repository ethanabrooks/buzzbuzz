#include <board.h>
#include <QDebug>

QList<Wall*> Board::walls;
int Board::boardSize = 500.0f;

Board::Board()
{
    this->numLights = 4;
    this->numMosquitoes = 500;
    this->numWalls = 1;
    this->frog = new Frog();
    this->mosquitoesCaught = 0;
    this->mosquitoesEaten = 0;
    //initialize Player and pass along lights & walls
    this->player = new MyPlayer();
    this->player->lights = this->lights;
    this->player->walls = this->walls;

    this->currRound = 0;
    this->maxRounds = 1000;
    this->captureTarget = (int) this->numMosquitoes;// * 0.50f; // initially capture 100%
}

void Board::initialize() {
    this->currRound = 0;
    this->mosquitoesEaten = 0;
    this->mosquitoesCaught = 0;
    this->mosquitoes.clear();
    this->lights.clear();
    //this->walls.clear();
    this->player->lights.clear();
    //this->player->walls.clear();


    for (int i = 0; i < this->numMosquitoes; i++) {
        float random = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/this->boardSize);
        float random2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/this->boardSize);
        Mosquito* m = new Mosquito();
        m->position = glm::vec2(random, random2);
        this->mosquitoes.append(m);
    }

    for (int j = 0; j < this->numLights; j++) {
        float random = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/this->boardSize);
        float random2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/this->boardSize);
        Light* l = new Light();
        l->moveTo(random, random2);
        this->lights.append(l);
    }
    this->player->lights = this->lights;
    generateBoardForPlayer();
    this->frog->position = this->player->initializeFrog(&this->playerBoard);
    this->player->initializeLights(&this->playerBoard);
}

bool checkWithinRadius(glm::vec2 objPos, glm::vec2 circlePos, int radius) {
    //(x - center_x)^2 + (y - center_y)^2 < radius^2
    //check if mosquitoes are caught and set them as "caught" within the light's radius
    float check = (objPos.x - circlePos.x) * (objPos.x - circlePos.x) + (objPos.y - circlePos.y) * (objPos.y - circlePos.y);
    if (check < (radius * radius)) {
        return true;
    } else {
        return false;
    }
}

void Board::generateBoardForPlayer() {
    QVector<QVector<int> > result;
    result.resize(this->boardSize+1);

    for (int v = 0; v < result.size(); v++) {
        result[v].resize(this->boardSize+1);
    }

    // initialize vector to 0s
    for (int a = 0; a < result.size(); a++) {
        for (int b = 0; b < result.at(a).size(); b++) {
            result[a][b] = 0;
            //qDebug() << "board[" << a << "][" << b << "] is " << result.at(a).at(b);
        }
    }

    for (int i = 0; i < this->mosquitoes.length(); i++) {
        int xPos = (int) glm::floor(this->mosquitoes.at(i)->position.x);
        int yPos = (int) glm::floor(this->mosquitoes.at(i)->position.y);

        if (xPos <= this->boardSize && yPos <= this->boardSize && xPos >= 0 && yPos >= 0) {
            result[xPos][yPos]++;
            //qDebug() << xPos << ", " << yPos << ": " << result[xPos][yPos];
        }
    }

    this->playerBoard = result;
}

void Board::step() {
    this->currRound++;

    if (this->lights.size() != this->player->lights.size()) {
        qDebug() << "Num of player lights not equal to num of board lights";
        return;
    }

    // Ask the player where the lights should move
    generateBoardForPlayer(); //update playerboard
    QList<glm::vec2> lightPositionsCheck;
    for (int l1 = 0; l1 < this->lights.length(); l1++) {
        lightPositionsCheck.append(this->lights.at(l1)->getPosition());
    }
    this->player->updateLights(&this->playerBoard);

    for (int l2 = 0; l2 < this->lights.length(); l2++) {
//        qDebug() << this->lights.at(l2)->position.x << this->lights.at(l2)->position.y;
//        qDebug() << lightPositionsCheck.at(l2).x << lightPositionsCheck.at(l2).y;
        if (glm::length(this->lights.at(l2)->getPosition() - lightPositionsCheck.at(l2)) > 3.0f) {
            qDebug() << "Detected that light" << l2 << "has been moved more than once in one step. Resetting to previous position.";
            // if lights have been moved more than once by the player in one step, set them to their previous position
            while (this->lights.at(l2)->getPosition().x - lightPositionsCheck.at(l2).x > 0 ||
                   this->lights.at(l2)->getPosition().y - lightPositionsCheck.at(l2).y > 0) {
                this->lights.at(l2)->returnToPreviousPosition();
            }
        }
    }

    // Move the mosquitoes
    for (int i = 0; i < this->mosquitoes.length(); i++) {
        // see if it's about to be eaten by the frog
        if (checkWithinRadius(this->mosquitoes.at(i)->position, this->frog->position, this->frog->radius)) {
            this->mosquitoes.at(i)->isEaten = true;
        }
        else {
            float closest = 500;
            glm::vec2 closestLightPos;
            this->mosquitoes.at(i)->isCaught = false; // this needs to be reset in case the light becomes obstructed
            for (int k = 0; k < this->lights.length(); k++) {

                // TODO: only consider lights that the mosquito can see and are not obstructed by a wall
                bool canSeeLight = true;
                for (int w = 0; w < this->walls.length(); w++) {
                    if (this->walls[w]->isInvalidMove(this->mosquitoes.at(i)->position, this->lights.at(k)->getPosition())) {
                        canSeeLight = false;
                        break;
                    }
                }

                if (canSeeLight) {
                    if (checkWithinRadius(this->mosquitoes.at(i)->position, this->lights.at(k)->getPosition(), this->lights.at(k)->radius)) {
                        this->mosquitoes.at(i)->isCaught = true;
                        this->mosquitoesCaught++;
                        closestLightPos = this->lights.at(k)->getPosition();
                        break;
                    } else {
                        glm::vec2 currDist = this->lights.at(k)->getPosition() - this->mosquitoes.at(i)->position;
                        if (glm::length(currDist) < closest) {
                            closest = glm::length(currDist);
                            closestLightPos = this->lights.at(k)->getPosition();
                        }
                    }
                }
            }

            glm::vec2 nextMove = this->mosquitoes.at(i)->calculateNextMove(closestLightPos);
            if (checkValidMove(this->mosquitoes.at(i)->position, nextMove)) this->mosquitoes.at(i)->move(nextMove);
        }
    }
}

void Board::updateMosquitoesEaten() {
    int result = 0;
    for (int i = 0; i < this->mosquitoes.length(); i++) {
        if (this->mosquitoes.at(i)->isEaten) {
            result++;
        }
    }
    this->mosquitoesEaten = result;
}


bool Board::checkValidMove(glm::vec2 oldPos, glm::vec2 newPos) {
    bool outsideBounds = false;
    if (newPos.x < 0.0f || newPos.y < 0.0f || newPos.x > Board::boardSize || newPos.y > Board::boardSize) outsideBounds = true;
    float moveLength = glm::length((newPos - oldPos));
    bool goesThroughWall = false;
    for (int w = 0; w < Board::walls.length(); w++) {
        if (Board::walls.at(w)->isInvalidMove(oldPos, newPos)) {
            goesThroughWall = true;
        }
    }

    if (goesThroughWall || outsideBounds || moveLength > 3.0f) {
        //if (moveLength > 3.0f) qDebug() << "Attempted to move more than one unit. Resetting to previous position.";
        return false;
    } else {
        return true;
    }
}

// definition included here because in c++, methods must be declared before we can use them
bool Light::moveTo(float newPosx, float newPosy) {
    glm::vec2 newPos = glm::vec2(newPosx, newPosy);
    if (Board::checkValidMove(this->position, newPos)) {
        this->position = newPos;
        this->trail.append(this->position);
        return true;
    } else {
        return false;
    }
}

void Light::moveRandomly() {
    float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*M_PI)));
    float deltax = 2.0f * cos(r);
    float deltay = 2.0f * sin(r);
    glm::vec2 newPos = glm::vec2(this->position.x + deltax, this->position.y + deltay);
    if (Board::checkValidMove(this->position, newPos)) {
        this->position = newPos;
        this->trail.append(this->position);
    }
}
