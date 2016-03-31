#include "light.h"
#include <QDebug>

Light::Light()
{
    this->radius = 100;
    int r =  250; //rand() % 255;
    int r1 = 50; //rand() % 255;
    int r2 = 250; //rand() % 255;
    this->trailColor = QColor(r, r1, r2);
    this->position = glm::vec2(0.0f,0.0f);
}

glm::vec2 Light::getPosition() {
    return glm::vec2(this->position);
}

void Light::setInitialPosition(float posx, float posy) {
    if (this->position == glm::vec2(0.0f, 0.0f)) {
        this->position = glm::vec2(posx, posy);
    }
    this->trail.append(this->position);
}

void Light::returnToPreviousPosition() {
    if (trail.size() > 1) {
        glm::vec2 saved = this->trail.takeLast();
//        qDebug() << this->position.x << this->position.y;
//        qDebug() << saved.x << saved.y;

        this->position = saved;
    } else {
        this->position = this->trail.first();
    }
}
