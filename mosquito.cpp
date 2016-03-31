#include "mosquito.h"

Mosquito::Mosquito()
{
    this->isCaught = false;
    this->isEaten = false;
}

float generateRandomAngle() {
    float result = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*M_PI)));
    return result;
}

float generateAngle(glm::vec2 nearestLightPos, glm::vec2 mosquitoPosition) {
    float angle = atan(abs(nearestLightPos.y - mosquitoPosition.y) / (nearestLightPos.x - mosquitoPosition.x)) * 180/M_PI;
    if (nearestLightPos.x > mosquitoPosition.x && nearestLightPos.y < mosquitoPosition.y) {
        return angle;
    } else if (nearestLightPos.x < mosquitoPosition.x && nearestLightPos.y > mosquitoPosition.y) {
        return angle + 180;
    } else if (nearestLightPos.x < mosquitoPosition.x && nearestLightPos.y < mosquitoPosition.y) {
        return 180 - angle;
    } else if (nearestLightPos.x > mosquitoPosition.x && nearestLightPos.y > mosquitoPosition.y) {
        return 360 - angle;
    } else {
        return angle;
    }
}

glm::vec2 Mosquito::calculateNextMove(glm::vec2 nearestLightPos) {
    if (isCaught) {
        glm::vec2 dir = nearestLightPos - this->position;
        glm::vec2 newPos = this->position + 0.01f*dir;
        float random = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(60)));
        float randomAngle = generateAngle(nearestLightPos, this->position) - 30 + random;
        float deltax = 2.0f * cos(randomAngle);
        float deltay = 2.0f * sin(randomAngle);
        float newx = newPos.x + deltax;
        float newy = newPos.y - deltay;

        return glm::vec2(newx, newy);
    } else {
        float randomAngle = generateRandomAngle();
        float deltax = 2.0f * cos(randomAngle);
        float deltay = 2.0f * sin(randomAngle);
        float newx = this->position.x + deltax;
        float newy = this->position.y - deltay;

        return glm::vec2(newx,newy);
    }
}

void Mosquito::move(glm::vec2 nextMove) {
    this->position = nextMove;
}
