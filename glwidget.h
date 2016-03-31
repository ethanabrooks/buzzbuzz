#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>

class Helper;

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(Helper *helper, QWidget *parent);

public slots:
    void animate();
    void step();
    void play();
    void stop();

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    Helper *helper;
    int elapsed;
    bool timerStopped;
    bool playPressed;
};

#endif
