#include "glwidget.h"
#include "helper.h"

#include <QPainter>
#include <QTimer>
#include <QDebug>
#include <QMessageBox>

GLWidget::GLWidget(Helper *helper, QWidget *parent)
    : QOpenGLWidget(parent), helper(helper)
{
    elapsed = 0;
    setFixedSize(500, 500);
    setAutoFillBackground(false);
    playPressed = false;
}

void GLWidget::animate()
{
    if (this->playPressed) {
        step();
    }
}

void GLWidget::step()
{
    if (this->helper->b->mosquitoesEaten >= this->helper->b->captureTarget) {
        this->playPressed = false;
        QMessageBox msgBox;
        msgBox.setWindowTitle("You Won");
        QString str = "You Won! \nResults: Caught " + QString::number(this->helper->b->mosquitoesEaten) + " mosquitoes, or " +
                QString::number((double) this->helper->b->mosquitoesEaten/this->helper->b->captureTarget * 100) + "% of the capture target in "
                + QString::number(this->helper->b->currRound) + " rounds.";
        msgBox.setText(str);
        msgBox.exec();
    }
    else if (this->helper->b->currRound >= this->helper->b->maxRounds) {
        this->playPressed = false;
        QMessageBox msgBox;
        msgBox.setWindowTitle("Game Over");
        QString str = "Game over! \nResults: Caught " + QString::number(this->helper->b->mosquitoesEaten) + " mosquitoes, or " +
                QString::number((double) this->helper->b->mosquitoesEaten/this->helper->b->captureTarget * 100) + "% of the capture target in " +
                QString::number(this->helper->b->currRound) + " rounds.";
        msgBox.setText(str);
        msgBox.exec();
    } else {
        this->helper->b->step();
        this->repaint();
    }
}

void GLWidget::play()
{
    this->playPressed = true;
}

void GLWidget::stop() {
    this->playPressed = false;
}

void GLWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    helper->paint(&painter, event, elapsed, this->timerStopped);
    painter.end();
}
