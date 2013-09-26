#include "segway.h"

#include <QStringList>

#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/ioctl.h>
#include <linux/input.h>


#define CMD_MAX                        100.0F
#define DEG2RAD                        0.01745329238F
#define EXEC_PERIOD                    0.00400000019F


Segway::Segway(QObject *parent) :
    QObject(parent),
    segwayState(INIT_MODE),
    averageCount(0)
{
    gyroOriginalTilts << 0 << 0 << 0;
    meanGyro << 0.0 << 0.0 << 0.0;

    keysFd = open("/dev/input/event0", O_SYNC, O_RDONLY);
    if (keysFd == -1)
    {
        qDebug()<<"cannot open keys input file reason: "<<errno;
    } else {
        keysSocket = QSharedPointer<QSocketNotifier>(new QSocketNotifier(keysFd, QSocketNotifier::Read, this));
        connect(keysSocket.data(), SIGNAL(activated(int)), this, SLOT(keysEvent()));
        keysSocket->setEnabled(true);
    }

    if (!infoServer.listen(QHostAddress::Any, 1234))
    {
        qDebug() << "unable to start the server:" << infoServer.errorString();
        infoServer.close();
        return;
    }
    connect(&infoServer, SIGNAL(newConnection()), this, SLOT(setConnection()));

}

void Segway::keysEvent()
{
    struct input_event event;

    if (read(keysFd, reinterpret_cast<char*>(&event), sizeof(event)) != sizeof(event))
    {
        qDebug()<<"keys: incomplete data read";
        return;
    }
    if (event.type == EV_KEY)
    {
        int keyCode = static_cast<int>(event.code);
        qDebug() << keyCode;
    }
}

void Segway::buttonPressed()
{
    switch (segwayState)
    {
        case INIT_MODE:
            brick.motor(1)->setPower(0);
            brick.motor(2)->setPower(0);
            //nxt_motor_set_count(PORT_MOTOR_L, 0); /* reset left motor count */
            //nxt_motor_set_count(PORT_MOTOR_R, 0); /* reset right motor count */

            resetToZero();
            segwayState = CALC_MODE;

            prepareTimer.singleShot(2000, this, SLOT(startStabilization()));
            connect(&deviceTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
            deviceTimer.start(10);
            break;
        case CALC_MODE:
        case CONTROL_MODE:
            segwayState = INIT_MODE;
//            resetToZero();
            deviceTimer.stop();
            disconnect(&deviceTimer, SIGNAL(timeout()), this, SLOT(stabilization()));
            break;
    }
}

void Segway::prepareSegway()
{
    QVector<int> temp;
    temp = brick.gyro()->readTilts();
    gyroOriginalTilts[0] += temp[0];
    gyroOriginalTilts[1] += temp[1];
    gyroOriginalTilts[2] += temp[2];
    ++averageCount;
}

void Segway::startStabilization()
{

    const float fcount = averageCount;
    meanGyro[0] = gyroOriginalTilts[0] / fcount;
    meanGyro[1] = gyroOriginalTilts[1] / fcount;
    meanGyro[2] = gyroOriginalTilts[2] / fcount;

    qDebug() << meanGyro;
    deviceTimer.stop();
    disconnect(&deviceTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
    segwayState = CONTROL_MODE;
    connect(&deviceTimer, SIGNAL(timeout()), this, SLOT(stabilization()));
    deviceTimer.start(4);
}

void Segway::stabilization()
{

    /*
    (void)ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
    * R/C command from NXT GamePad
    * buf[0]: -100(forward max.) to 100(backward max.)
    * buf[1]: -100(turn left max.) to 100(turn right max.)
    */
    cmd_forward = 0;
    cmd_turn = 0;
    /*
    cmd_forward = -(S8)bt_receive_buf[0];
//    reverse the direction
    cmd_turn = (S8)bt_receive_buf[1];
    if (obstacle_flag == 1){
//        make NXTway-GS move backward to avoid obstacle
        cmd_forward = -100;
        cmd_turn = 0;
    }
    */
    /* NXTway-GS C API balance control function (has to be invoked in 4msec period) */

    balance_control(/*
        (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
        (F32)gyro_offset,
        (F32)nxt_motor_get_count(PORT_MOTOR_L),
        (F32)nxt_motor_get_count(PORT_MOTOR_R),
        (F32)ecrobot_get_battery_voltage()
        */);

//    brick.motor(1)->setPower(pwm_l);
//    brick.motor(2)->setPower(pwm_r);

    //log data
}

void Segway::balance_control()
{

}

void Segway::resetToZero()
{
    gyroOriginalTilts[0] = 0;
    gyroOriginalTilts[1] = 0;
    gyroOriginalTilts[2] = 0;
    meanGyro[0] = 0.0;
    meanGyro[1] = 0.0;
    meanGyro[2] = 0.0;
    averageCount = 0;

    ud_err_theta = 0.0F;
    ud_theta_ref = 0.0F;
    ud_thetadot_cmd_lpf = 0.0F;
    ud_psi = 0.0F;
    ud_theta_lpf = 0.0F;

    pwm_l = 0.0;
    pwm_r = 0.0;
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
