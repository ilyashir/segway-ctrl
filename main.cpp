// compiled with trik-sdk at 17.10.2013

#include <QtGui/QApplication>
#include <QMetaType>
#include "segway.h"

int main(int argc, char *argv[])
{
    qRegisterMetaType<QPair<float,float>>("QPair<float,float>");
    QApplication a(argc, argv);


    QString configPath = "./";
    if (a.arguments().contains("-c"))
    {
        int const index = a.arguments().indexOf("-c");
        if (a.arguments().count() <= index + 1)
        {
            //        printUsage();
            return 1;
        }

        configPath = a.arguments()[index + 1];
        if (configPath.right(1) != "/")
        {
            configPath += "/";
        }
    }

    Segway ctrl(a.thread(),configPath);
    ctrl.grabKeyboard();
    return a.exec();
}
