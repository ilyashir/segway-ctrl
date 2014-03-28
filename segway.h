#pragma once

#include <QObject>
#include <QWidget>
#include <QTcpSocket>
#include <QTcpServer>
#include <QTimer>
#include <QSocketNotifier>
#include <QSharedPointer>
#include <trikControl/brick.h>
#include <QVector3D>
#include <QMatrix4x4>
#include "encodersReader.h"
#include "stabilizer.h"
using namespace trikControl;

#define KEY_ENTER 28
#define KEY_ESC 139

class Segway : public QWidget
{
    Q_OBJECT
public:
    explicit Segway(QThread *guiThread, QString configPath);
    ~Segway();
    inline QPair<float, float> readEncoders() { return qMakePair(brick.encoder("JB4")->read(), brick.encoder("JB3")->read()); }
protected:
    void resetToZero();
    void balance_control();
    void buttonPressed();
//    virtual void keyPressEvent(QKeyEvent *event);

//    EncodersReader encodersReader;

//    Stabilizer stabilizer;

signals:

private slots:
    void stabilization();
    void getButton(int code, int value);
    void setConnection();
    void readInfoData();
    void startStabilization();
    void prepareSegway();
    void getVoltage();
    void encodersReady(QPair<float, float> data);
    void stabilizationComplete() {}

private:    
    Brick brick;
    QTcpSocket *infoSocket;
    QTcpServer infoServer;
    QTimer initTimer;
    QTimer batteryTimer; //100ms
    QTimer taskTimer; //4ms


    QSharedPointer<QSocketNotifier> keysSocket;
    int keysFd;

    QVector3D gyroOriginalTilts;
    QVector3D gyroOffsetTilts;
    QVector3D accOriginal;
    qreal acc;
    QVector3D alpha_acc;
    QMatrix4x4 accRotate;

    enum { INIT_MODE,
           CALC_MODE,
           CONTROL_MODE
    } segwayState;

    int averageCount; /* average count to calc gyro offset */

    float args_battery;
    float args_theta_m_l;
    float args_theta_m_r;

    float ud_err_psi;
    float ud_err_theta;
    float ud_theta_ref;
    float ud_thetadot_cmd_lpf;
    float ud_psi;
    float ud_theta_lpf;

    float cmd_forward;
    float cmd_turn;
    float pwm_l;
    float pwm_r;

    float K_F[4];
    float K_I;
    float KK_I;


};
