#include "window.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <execinfo.h>
#include <cxxabi.h>

int main(int argc, char *argv[])
{

    srand (static_cast <unsigned> (time(0)));

    QApplication app(argc, argv);

    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    Window window;
    window.show();
    return app.exec();
}
