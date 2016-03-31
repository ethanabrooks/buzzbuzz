#ifndef WINDOW_H
#define WINDOW_H

#include "helper.h"
#include "glwidget.h"

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QPushButton>

class Window : public QWidget
{
    Q_OBJECT

public:
    Window();

private slots:
    void stop();
    void play();
    void updateText();
    void startNewGame();
    void changeTimerDelay();
    void updateCaptureTarget();
    void updateMaxRounds();

private:
    Helper helper;
    QTimer* timer;
    QLabel* mosquitoLabel;
    QLabel* roundLabel;
    GLWidget* openGL;
    QSlider* timeDelaySlider;
    QSpinBox* captureTargetSpinBox;
    QSpinBox* maxRoundsSpinBox;
    int timerDelay;
    QPushButton* stepButton;
    QPushButton* playButton;
    QPushButton* pauseButton;
    QPushButton* newGameButton;
    void setWalls();
};

#endif
