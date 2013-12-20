#include <QtGui/QApplication>
#include <QMetaType>
#include "segway.h"

int main(int argc, char *argv[])
{
    qRegisterMetaType<QPair<float,float>>("QPair<float,float>");
    QApplication a(argc, argv);
    Segway ctrl(a.thread());
    ctrl.grabKeyboard();
    return a.exec();
}
