#ifndef SEGWAY_H
#define SEGWAY_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QTimer>

#include <trikControl/brick.h>

using namespace trikControl;

class Segway : public QObject
{
    Q_OBJECT
public:
    explicit Segway(QObject *parent = 0);

protected:
    void resetToZero();

signals:

private slots:
    void buttonPressed();

    void setConnection();
    void readInfoData();
    void startStabilization();
    void prepareSegway();
    void stabilization();

private:
    Brick brick;
    QTcpSocket *infoSocket;
    QTcpServer infoServer;
    QTimer prepareTimer;
    QTimer deviceTimer;

    QTimer TESTTIMER;

    QVector<int> accelOriginalTilts;
    QVector<int> gyroOriginalTilts;
    enum { stopped,
           preparing,
           balancing
    }segwaystate;

    QVector<float> meanGyro;

    int count;
};

#endif // SEGWAY_H
