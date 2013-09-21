#include <QtCore/QCoreApplication>
#include "segway.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Segway ctrl;

    return a.exec();
}
