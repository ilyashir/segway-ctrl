#include "segway.h"

#include <QStringList>

Segway::Segway(QObject *parent) :
    QObject(parent),
    segwaystate(stopped),
    count(0)
{
    accelOriginalTilts << 0 << 0 << 0;
    gyroOriginalTilts << 0 << 0 << 0;

    if (!infoServer.listen(QHostAddress::Any, 1234))
    {
        qDebug() << "unable to start the server:" << infoServer.errorString();
        infoServer.close();
        return;
    }
    qDebug() << "server starts";
    connect(&infoServer, SIGNAL(newConnection()), this, SLOT(setConnection()));

    TESTTIMER.start(4000);
    connect(&TESTTIMER, SIGNAL(timeout()), this, SLOT(buttonPressed()));
}

void Segway::buttonPressed()
{
    switch (segwaystate)
    {
        case balancing:
            segwaystate = stopped;
            resetToZero();
            deviceTimer.stop();
            disconnect(&deviceTimer, SIGNAL(timeout()), this, SLOT(stabilization()));
            break;
        case stopped:
            segwaystate = preparing;
            prepareTimer.singleShot(2000, this, SLOT(startStabilization()));
            connect(&deviceTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
            deviceTimer.start(10);
            break;
        case preparing:
            segwaystate = stopped;
            break;
    }
}

void Segway::prepareSegway()
{
    /*
    QVector<int> temp;
    temp = brick.gyro()->readTilts();
    gyroOriginalTilts[0] += temp[0];
    gyroOriginalTilts[1] += temp[1];
    gyroOriginalTilts[2] += temp[2];
    */
    ++count;
}

void Segway::startStabilization()
{
    const float fcount = count;
    meanGyro[0] = gyroOriginalTilts[0] / fcount;
    meanGyro[1] = gyroOriginalTilts[1] / fcount;
    meanGyro[2] = gyroOriginalTilts[2] / fcount;

    deviceTimer.stop();
    qDebug() << "start stabilization";
    disconnect(&deviceTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
    connect(&deviceTimer, SIGNAL(timeout()), this, SLOT(stabilization()));
//    deviceTimer.start(50);
}

void Segway::stabilization()
{
    qDebug() << ++count;

}

void Segway::resetToZero()
{
    accelOriginalTilts[0] = 0;
    accelOriginalTilts[1] = 0;
    accelOriginalTilts[2] = 0;
    gyroOriginalTilts[0] = 0;
    gyroOriginalTilts[1] = 0;
    gyroOriginalTilts[2] = 0;
    count = 0;
}

void Segway::setConnection()
{
    qDebug() << "new connection";

    infoSocket = infoServer.nextPendingConnection();
    connect(infoSocket, SIGNAL(disconnected()), infoSocket, SLOT(deleteLater()));
    connect(infoSocket, SIGNAL(readyRead()), this, SLOT(readInfoData()));
}

void Segway::readInfoData()
{
    QDataStream input(infoSocket);
    input.setVersion(QDataStream::Qt_4_2);
    QString text;
    input >> text;
    QStringList list = text.split(" ");

    foreach (QString const &args, list)
    {
        qDebug() << args.toInt();
    }
}
