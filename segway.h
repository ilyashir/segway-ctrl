#ifndef SEGWAY_H
#define SEGWAY_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QTimer>
#include <QSocketNotifier>
#include <QSharedPointer>
#include <trikControl/brick.h>

using namespace trikControl;

class Segway : public QObject
{
    Q_OBJECT
public:
    explicit Segway(QObject *parent = 0);

protected:
    void resetToZero();
    void balance_control();
    void buttonPressed();

signals:

private slots:
    void keysEvent();
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

    QSharedPointer<QSocketNotifier> keysSocket;
    int keysFd;

    QVector<int> gyroOriginalTilts;
    QVector<float> meanGyro;

    enum { INIT_MODE,
           CALC_MODE,
           CONTROL_MODE
    } segwayState;

    int averageCount; /* average count to calc gyro offset */

    float ud_err_theta;
    float ud_theta_ref;
    float ud_thetadot_cmd_lpf;
    float ud_psi;
    float ud_theta_lpf;

    float cmd_forward;
    float cmd_turn;
    float pwm_l;
    float pwm_r;
};

#endif // SEGWAY_H
