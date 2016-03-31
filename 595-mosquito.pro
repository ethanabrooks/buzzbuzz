#-------------------------------------------------
#
# Project created by QtCreator 2015-10-23T13:39:45
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 595-mosquito
TEMPLATE = app


SOURCES += main.cpp\
    glwidget.cpp \
    window.cpp \
    mosquito.cpp \
    helper.cpp \
    light.cpp \
    board.cpp \
    wall.cpp \
    frog.cpp \
    player.cpp \
    myplayer.cpp \
    matrix.cpp

HEADERS  += \
    glwidget.h \
    window.h \
    mosquito.h \
    helper.h \
    light.h \
    board.h \
    wall.h \
    frog.h \
    player.h \
    myplayer.h \
    matrix.h

FORMS    += \
    mainwindow.ui

INCLUDEPATH += include

DISTFILES += \
    wall_setup.txt

LIBS += -larmadillo

macx {
 QMAKE_CXXFLAGS += -std=c++11

    _ARMADILLO_PATH = /usr/local/Cellar/armadillo/6.500.5/
    INCLUDEPATH += "$${_ARMADILLO_PATH}/include/"
    LIBS += -L$${_ARMADILLO_PATH}/lib
    ## Use only one of these:
    LIBS += -larmadillo # using dynamic lib (not sure if you need that "-mt" at the end or not)
    #LIBS += $${_BOOST_PATH}/lib/libboost_chrono-mt.a # using static lib
}
