#include "qt_4dof.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qt_4dof w;
    w.show();
    return a.exec();
}
