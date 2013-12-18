#include <QtGui/QApplication>
#include "segway.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Segway ctrl(a.thread());
    ctrl.grabKeyboard();
    return a.exec();
}
