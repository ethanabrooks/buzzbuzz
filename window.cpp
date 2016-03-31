#include "glwidget.h"
#include "window.h"

#include <QGridLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QSlider>
#include <QSpinBox>
#include <QMessageBox>
#include <QFile>
#include <QFileInfo>

Window::Window()
{
    this->timerDelay = 250;
    setWalls();
    this->helper.b->initialize();
    //setWalls();
    setWindowTitle(QString("Mosquito Buzz Buzz: " + this->helper.b->player->playerName));

    this->openGL = new GLWidget(&this->helper, this);

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(openGL, 0, 1);
    //layout->addWidget(openGLLabel, 1, 1);
    setLayout(layout);

    this->mosquitoLabel = new QLabel(this);
    layout->addWidget(this->mosquitoLabel, 1, 1);
    mosquitoLabel->setText("Mosquitoes Eaten: 0");
    mosquitoLabel->setAlignment(Qt::AlignLeft);

    this->roundLabel = new QLabel(this);
    layout->addWidget(this->roundLabel, 1, 1);
    roundLabel->setText("Round: 0");
    roundLabel->setAlignment(Qt::AlignHCenter);

    this->timer = new QTimer(this);
    connect(this->timer, SIGNAL(timeout()), openGL, SLOT(animate()));
    connect(this->timer, SIGNAL (timeout()), this, SLOT(updateText()));
    this->timer->start(this->timerDelay);

    this->stepButton = new QPushButton("Step", this);
    layout->addWidget(stepButton, 3, 1);
    layout->setAlignment(stepButton, Qt::AlignTop);
    connect(stepButton, SIGNAL (released()), openGL, SLOT (step()));
    connect(stepButton, SIGNAL (released()), this, SLOT(updateText()));

    this->playButton = new QPushButton("Play", this);
    this->playButton->setMinimumWidth(250);
    layout->addWidget(playButton, 4, 1);
    layout->setAlignment(playButton, Qt::AlignLeft);
    connect(playButton, SIGNAL (released()), openGL, SLOT (play()));
    connect(playButton, SIGNAL (released()), this, SLOT(play()));

    this->pauseButton = new QPushButton("Pause", this);
    this->pauseButton->setMinimumWidth(250);
    layout->addWidget(pauseButton, 4, 1);
    layout->setAlignment(pauseButton, Qt::AlignRight);
    connect(pauseButton, SIGNAL (released()), this, SLOT (stop()));
    connect(pauseButton, SIGNAL (released()), openGL, SLOT (stop()));

    this->newGameButton = new QPushButton("Begin New Game", this);
    layout->addWidget(newGameButton, 1, 1);
    layout->setAlignment(newGameButton, Qt::AlignRight);
    connect(newGameButton, SIGNAL (released()), this, SLOT(startNewGame()));

    this->timeDelaySlider = new QSlider(Qt::Horizontal, this);
    this->timeDelaySlider->setFixedWidth(400);
    this->timeDelaySlider->setMinimum(0);
    this->timeDelaySlider->setSingleStep(10);
    this->timeDelaySlider->setValue(250);
    this->timeDelaySlider->setMaximum(800);
    this->timeDelaySlider->setSingleStep(10);
    connect(this->timeDelaySlider, SIGNAL (sliderReleased()), this, SLOT(changeTimerDelay()));
    layout->addWidget(this->timeDelaySlider, 5, 1);
    layout->setAlignment(this->timeDelaySlider, Qt::AlignRight);

    QLabel* timerLabel = new QLabel(this);
    layout->addWidget(timerLabel, 5, 1);
    layout->setAlignment(timerLabel, Qt::AlignLeft);
    timerLabel->setText("Timer Delay");
    timerLabel->setIndent(10);
    timerLabel->setBuddy(this->timeDelaySlider);

    QLabel* captureTargetLabel = new QLabel(this);
    layout->addWidget(captureTargetLabel, 2, 1);
    captureTargetLabel->setIndent(65);
    captureTargetLabel->setText("Capture Target (%)");

    this->captureTargetSpinBox = new QSpinBox; // set percentage for capture rate needed to win
    captureTargetSpinBox->setRange(1, 100);
    captureTargetSpinBox->setValue(100);
    layout->addWidget(captureTargetSpinBox, 2, 1);
    layout->setAlignment(captureTargetSpinBox, Qt::AlignLeft);
    connect(captureTargetSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateCaptureTarget()));

    QLabel* roundsLabel = new QLabel(this);
    layout->addWidget(roundsLabel, 2, 1);
    layout->setAlignment(roundsLabel, Qt::AlignCenter);
    roundsLabel->setIndent(170);
    roundsLabel->setText("Rounds (max)");

    this->maxRoundsSpinBox = new QSpinBox; // set max num of rounds before game over
    maxRoundsSpinBox->setRange(100, 10000);
    maxRoundsSpinBox->setValue(1000);
    layout->addWidget(maxRoundsSpinBox, 2, 1);
    layout->setAlignment(maxRoundsSpinBox, Qt::AlignCenter);
    connect(maxRoundsSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateMaxRounds()));
}

void Window::setWalls() {
    this->helper.b->walls.clear();
    this->helper.b->player->walls.clear();

    QFile file("wall_setup.txt");
    file.open(QIODevice::ReadOnly);

    if(!file.isOpen()){
        qDebug() << "WARNING: Could not read wall_setup.txt" << endl << "See assignment description for instructions on how to fix this" << endl;
        this->helper.b->initialize();
        return;
    }

    QTextStream in(&file);
    int wallsCount = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList bits = line.split(",");
        if (bits.size() == 1) {
            // get number of walls
            this->helper.b->numWalls = bits.at(0).trimmed().toInt();
        } else {
            // get wall endpoints (MUST HAVE 4 in each )
            if (bits.size() != 4) {
                qDebug() << "Wall endpoints incorrectly specified";
                return;
            } else {
                if (wallsCount < this->helper.b->numWalls) {
                    float currVal = bits.at(0).trimmed().toFloat();
                    float currVal2 = bits.at(1).trimmed().toFloat();
                    float currVal3 = bits.at(2).trimmed().toFloat();
                    float currVal4 = bits.at(3).trimmed().toFloat();
                    //qDebug() << currVal << currVal2 << currVal3 << currVal4;
                    this->helper.b->walls.append(new Wall(glm::vec2(currVal, currVal2), glm::vec2(currVal3, currVal4)));
                    this->helper.b->player->walls.append(new Wall(glm::vec2(currVal, currVal2), glm::vec2(currVal3, currVal4)));
                    wallsCount++;
                }
            }
        }
    }

    if (wallsCount < this->helper.b->numWalls) {
        qDebug() << "You specified " << this->helper.b->numWalls << " wall(s) but only provided endpoints for " << wallsCount;
        qDebug() << "Using " << wallsCount << " wall(s)";
        this->helper.b->numWalls = wallsCount;
    }

//    for (int i = 0; i < this->helper.b->walls.size(); i++) {
//        qDebug() << this->helper.b->walls.at(i)->position.x << this->helper.b->walls.at(i)->position.y;
//    }
}

void Window::stop() {
    if (this->timer->isActive()) {
        this->timer->stop();
    }
}

void Window::play() {
    this->timer->start(this->timerDelay);
}

void Window::updateText() {
    //qDebug() << "mosquitoes captured " << this->helper.b->mosquitoesEaten;
    this->helper.b->updateMosquitoesEaten();
    QString str = "Mosquitoes Eaten: " + QString::number(this->helper.b->mosquitoesEaten) + " / " + QString::number(this->helper.b->captureTarget);
    this->mosquitoLabel->setText(str);
    QString str2 = "Round: " + QString::number(this->helper.b->currRound) + " / " + QString::number(this->helper.b->maxRounds);
    this->roundLabel->setText(str2);

    if (this->helper.b->currRound >= this->helper.b->maxRounds || this->helper.b->mosquitoesEaten >= this->helper.b->captureTarget) {
        this->stepButton->setEnabled(false);
        this->playButton->setEnabled(false);
        this->pauseButton->setEnabled(false);
        this->captureTargetSpinBox->setEnabled(false);
        this->maxRoundsSpinBox->setEnabled(false);
    }
}

void Window::updateCaptureTarget() {
    this->helper.b->captureTarget = this->helper.b->numMosquitoes * this->captureTargetSpinBox->value() / 100;
    updateText();
}

void Window::updateMaxRounds() {
    this->helper.b->maxRounds = this->maxRoundsSpinBox->value();
    updateText();
}

void Window::startNewGame() {
    setWalls();
    this->helper.b->initialize();
    //setWalls();
    this->openGL->repaint();
    updateText();
    this->stepButton->setEnabled(true);
    this->playButton->setEnabled(true);
    this->pauseButton->setEnabled(true);
    this->captureTargetSpinBox->setEnabled(true);
    this->maxRoundsSpinBox->setEnabled(true);
}

void Window::changeTimerDelay() {
    this->timerDelay = this->timeDelaySlider->sliderPosition();
    this->timer->start(this->timerDelay);
}
