#ifndef HELPER_H
#define HELPER_H

#include <QBrush>
#include <QFont>
#include <QPen>
#include <QWidget>
#include "board.h"

class Helper
{
public:
    Helper();

public:
    void paint(QPainter *painter, QPaintEvent *event, int elapsed, bool timerStopped);
    Board* b;

private:
    QBrush background;
    QBrush circleBrush;
    QPen circlePen;

    void drawFrog(QPainter *painter, float px, float py, float radius);
    void drawLight(QPainter *painter, float px, float py, float radius);
    void drawMosquito(QPainter *painter, float px, float py);
    void drawWall(QPainter *painter, float px1, float py1, float p1x, float p1y);
    void drawTrail(QPainter *painter, QList<glm::vec2> points, QColor trailColor);
    void drawDot(QPainter *painter, float px, float py, float radius);
};

#endif
