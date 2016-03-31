#include "helper.h"

#include <QPainter>
#include <QPaintEvent>
#include <QWidget>
#include <QDebug>

Helper::Helper()
{
    background = QBrush(QColor(64, 32, 64));
    circlePen = QPen(Qt::white);
    circlePen.setWidth(1);
    b = new Board();
}

void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed, bool timerStopped)
{

    painter->fillRect(event->rect(), background);

    for (int i = 0; i < b->lights.length(); i++) {
        glm::vec2 position = b->lights.at(i)->getPosition();
        drawLight(painter, position.x, position.y, b->lights.at(i)->radius);
        drawTrail(painter, b->lights.at(i)->trail, b->lights.at(i)->trailColor);
    }

    for (int j = 0; j < b->mosquitoes.length(); j++) {
        if (!b->mosquitoes.at(j)->isEaten) {
            drawMosquito(painter, b->mosquitoes.at(j)->position.x, b->mosquitoes.at(j)->position.y);
        }
    }

    for (int k = 0; k < b->walls.length(); k++) {
        drawWall(painter, b->walls.at(k)->point1.x, b->walls.at(k)->point1.y, b->walls.at(k)->point2.x, b->walls.at(k)->point2.y);
    }

    drawFrog(painter, b->frog->position.x, b->frog->position.y, b->frog->radius);
}

void Helper::drawFrog(QPainter *painter, float px, float py, float radius) {
    circlePen = QPen(Qt::green);
    circlePen.setWidth(2);
    painter->setPen(circlePen);
    painter->setBrush(QBrush(Qt::green));
    painter->drawEllipse(QPointF(px, py), radius, radius);
}



void Helper::drawLight(QPainter *painter, float px, float py, float radius) {
    circlePen = QPen(Qt::yellow);
    circlePen.setWidth(1);
    painter->setPen(circlePen);
    painter->drawPoint(px, py);
    painter->drawEllipse(QPointF(px, py), radius, radius);
}

void Helper::drawMosquito(QPainter *painter, float px, float py) {
    circlePen = QPen(Qt::white);
    circlePen.setWidth(1);
    painter->setPen(circlePen);
    float mosquitoSize = 5.0f;
    painter->drawLine(QPointF(px + mosquitoSize, py + mosquitoSize), QPoint(px - mosquitoSize, py - mosquitoSize));
    painter->drawLine(QPointF(px - mosquitoSize, py + mosquitoSize), QPoint(px + mosquitoSize, py - mosquitoSize));
}

void Helper::drawWall(QPainter *painter, float px, float py, float p1x, float p1y) {
    circlePen = QPen(Qt::red);
    circlePen.setWidth(2);
    painter->setPen(circlePen);

    painter->drawLine(QPointF(px, py), QPoint(p1x, p1y));
}

void Helper::drawTrail(QPainter *painter, QList<glm::vec2> points, QColor trailColor) {
    circlePen = QPen(trailColor);
    circlePen.setWidth(1);
    painter->setPen(circlePen);

   for (int i = 0; i < points.length() - 1; i += 2) {
        painter->drawLine(QPoint(points.at(i).x, points.at(i).y), QPoint(points.at(i+1).x, points.at(i+1).y));
    }
}

void Helper::drawDot(QPainter *painter, float px, float py, float radius) {
    circlePen = QPen(Qt::blue);
    circlePen.setWidth(2);
    painter->setPen(circlePen);
    painter->setBrush(QBrush(Qt::blue));
    painter->drawEllipse(QPointF(px, py), radius, radius);
}
