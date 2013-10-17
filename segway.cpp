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
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )

const float A_R = 0.996F; /* low pass filter gain for motors target count */
const float A_D = 0.8F; /* low pass filter gain for motors average count */
const float gyroConst = 820.846931; /*Parrots to rad*/
const float K_THETADOT = 6.25F;  /* 0.3/R*/
const float K_PHIDOT = 25.0F;  /* turn target speed gain */
const float BATTERY_GAIN = 0.018504035F;	/* battery voltage gain for motor PWM outputs */
const float BATTERY_OFFSET = 0.2112102855F;	/* battery voltage offset for motor PWM outputs */

Segway::Segway(QObject *parent) :
    QObject(parent),
    segwayState(INIT_MODE),
    averageCount(0)
{
    K_F[0] = -0.783855020175513;
    K_F[1] = -27.8085449316292;
    K_F[2] = -0.908246261210228;
    K_F[3] = -2.12746939876327;
    K_I = -0.447213595499757;
    qDebug() << "INIT_MODE";

    gyroOriginalTilts << 0 << 0 << 0;
    gyroOffsetTilts << 0.0 << 0.0 << 0.0;

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
    connect(&batteryTimer, SIGNAL(timeout()), this, SLOT(getVoltage()));
}

Segway::~Segway()
{
    batteryTimer.stop();
    taskTimer.stop();
    close(keysFd);

    brick.powerMotor("3")->setPower(0);
    brick.powerMotor("4")->setPower(0);
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
        int keyValue = static_cast<int>(event.value);
        if ((keyCode == KEY_F3) && (keyValue == 1))
        {
            buttonPressed();
        }
    }
}

void Segway::buttonPressed()
{
    switch (segwayState)
    {
        case INIT_MODE:
            resetToZero();
            brick.powerMotor("3")->setPower(0); //right motor
            brick.powerMotor("4")->setPower(0); //left motor
            brick.encoder(3)->reset(); //left encoder
            brick.encoder(4)->reset(); //right encoder

            segwayState = CALC_MODE;
            qDebug() << "CALC_MODE";

            initTimer.singleShot(2000, this, SLOT(startStabilization()));

            connect(&taskTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
            taskTimer.start(4);
            break;
        case CALC_MODE:
        case CONTROL_MODE:
            segwayState = INIT_MODE;
            brick.powerMotor("3")->setPower(0); //right motor
            brick.powerMotor("4")->setPower(0); //left motor
            qDebug() << "INIT_MODE";

            taskTimer.stop();
                batteryTimer.stop();
            disconnect(&taskTimer, SIGNAL(timeout()), this, SLOT(stabilization()));
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

void Segway::getVoltage()
{
    float voltage = brick.battery()->readVoltage();
    args_battery = voltage * 1000; //mV
}

void Segway::startStabilization()
{
    taskTimer.stop();

    const float fcount = averageCount;

    gyroOffsetTilts[0] = gyroOriginalTilts[0] / (fcount * gyroConst);
    gyroOffsetTilts[1] = gyroOriginalTilts[1] / (fcount * gyroConst);
    gyroOffsetTilts[2] = gyroOriginalTilts[2] / (fcount * gyroConst);

    args_battery = brick.battery()->readVoltage();

//    qDebug() << "gyro offset:" << gyroOffsetTilts;
    segwayState = CONTROL_MODE;
    qDebug() << "CONTROL_MODE";

    disconnect(&taskTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
    connect(&taskTimer, SIGNAL(timeout()), this, SLOT(stabilization()));

    batteryTimer.start(20);
    taskTimer.start(4);
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
    cmd_forward = -(S8)bt_receive_buf[0];  reverse the direction
    cmd_turn = (S8)bt_receive_buf[1];
    */

    QVector<int> temp = brick.gyro()->readTilts();
    gyroOriginalTilts[0] = temp[0];

    args_theta_m_l = brick.encoder(3)->get();
    args_theta_m_r = - brick.encoder(4)->get();

//    qDebug("left encoder: %f right encoder: %f", args_theta_m_l, args_theta_m_r);

    balance_control();

    int p_l = (int)pwm_l;
    int p_r = (int)pwm_r;

//    qDebug("pwmL: %d pwmR: %d", p_l, p_r);

    brick.powerMotor("3")->setPower(p_l);
    brick.powerMotor("4")->setPower(p_r);
}

void Segway::balance_control()
{
    float tmp_theta;
    float tmp_theta_lpf;
    float tmp_pwm_r_limiter;
    float tmp_psidot;
    float tmp_pwm_turn;
    float tmp_pwm_l_limiter;
    float tmp_thetadot_cmd_lpf;
    float tmp[4];
    float tmp_theta_0[4];

    /* Sum: '<S8>/Sum' incorporates:
      *  Constant: '<S3>/Constant6'
      *  Constant: '<S8>/Constant'
      *  Constant: '<S8>/Constant1'
      *  Gain: '<S3>/Gain1'
      *  Gain: '<S8>/Gain2'
      *  Inport: '<Root>/cmd_forward'
      *  Product: '<S3>/Divide'
      *  Product: '<S8>/Product'
      *  Sum: '<S8>/Sum1'
      *  UnitDelay: '<S8>/Unit Delay'
      */

    tmp_thetadot_cmd_lpf = (((cmd_forward / CMD_MAX) * K_THETADOT) * (1.0F - A_R)) + (A_R * ud_thetadot_cmd_lpf);
//    вся штука = 0

     /* Gain: '<S4>/Gzain' incorporates:
      *  Gain: '<S4>/deg2rad'
      *  Gain: '<S4>/deg2rad1'
      *  Inport: '<Root>/theta_m_l'
      *  Inport: '<Root>/theta_m_r'
      *  Sum: '<S4>/Sum1'
      *  Sum: '<S4>/Sum4'
      *  Sum: '<S4>/Sum6'
      *  UnitDelay: '<S10>/Unit Delay'
      */
     tmp_theta = (args_theta_m_l + args_theta_m_r) * 0.5F + ud_psi;

//     ud_psi=0
//     среднее показание с энкодеров

     /* Sum: '<S11>/Sum' incorporates:
      *  Constant: '<S11>/Constant'
      *  Constant: '<S11>/Constant1'
      *  Gain: '<S11>/Gain2'
      *  Product: '<S11>/Product'
      *  Sum: '<S11>/Sum1'
      *  UnitDelay: '<S11>/Unit Delay'
      */

     tmp_theta_lpf = ((1.0F - A_D) * tmp_theta) + (A_D * ud_theta_lpf);
//     фильтруем

     /* Gain: '<S4>/deg2rad2' incorporates:
      *  Inport: '<Root>/gyro'
      *  Inport: '<Root>/gyro_offset'
      *  Sum: '<S4>/Sum2'
      */

    tmp_psidot = (gyroOriginalTilts[0] / gyroConst) - gyroOffsetTilts[0];
//    qDebug("gyro delta: %f", tmp_psidot);

     /* Gain: '<S2>/Gain' incorporates:
      *  Constant: '<S3>/Constant2'
      *  Constant: '<S3>/Constant3'
      *  Constant: '<S6>/Constant'
      *  Constant: '<S9>/Constant'
      *  Gain: '<S1>/FeedbackGain'
      *  Gain: '<S1>/IntegralGain'
      *  Gain: '<S6>/Gain3'
      *  Inport: '<Root>/battery'
      *  Product: '<S2>/Product'
      *  Product: '<S9>/Product'
      *  Sum: '<S1>/Sum2'
      *  Sum: '<S1>/sum_err'
      *  Sum: '<S6>/Sum2'
      *  Sum: '<S9>/Sum'
      *  UnitDelay: '<S10>/Unit Delay'
      *  UnitDelay: '<S11>/Unit Delay'
      *  UnitDelay: '<S5>/Unit Delay'
      *  UnitDelay: '<S7>/Unit Delay'
      */

    tmp[0] = ud_theta_ref;
    tmp[1] = 0.0F;
    tmp[2] = tmp_thetadot_cmd_lpf;
    tmp[3] = 0.0F;
    tmp_theta_0[0] = tmp_theta;
    tmp_theta_0[1] = ud_psi;
    tmp_theta_0[2] = (tmp_theta_lpf - ud_theta_lpf) / EXEC_PERIOD;
    tmp_theta_0[3] = tmp_psidot;
    tmp_pwm_r_limiter = 0.0F;

    for (int i = 0; i < 4; i++) {
        tmp_pwm_r_limiter += (tmp[i] - tmp_theta_0[i]) * K_F[i];
    }

    tmp_pwm_r_limiter = (((K_I * ud_err_theta) + tmp_pwm_r_limiter) /
                                ((BATTERY_GAIN * args_battery) - BATTERY_OFFSET)) * 100.0F;

     /* Gain: '<S3>/Gain2' incorporates:
      *  Constant: '<S3>/Constant1'
      *  Inport: '<Root>/cmd_turn'
      *  Product: '<S3>/Divide1'
      */
     tmp_pwm_turn = (cmd_turn / CMD_MAX) * K_PHIDOT;

     /* Sum: '<S2>/Sum' */
     tmp_pwm_l_limiter = tmp_pwm_r_limiter + tmp_pwm_turn;

     /* Saturate: '<S2>/pwm_l_limiter' */
     tmp_pwm_l_limiter = rt_SATURATE(tmp_pwm_l_limiter, -100.0F, 100.0F);

     /* Outport: '<Root>/pwm_l' incorporates:
      *  DataTypeConversion: '<S1>/Data Type Conversion'
      */
     pwm_l = tmp_pwm_l_limiter;

     tmp_pwm_r_limiter -= tmp_pwm_turn;

     tmp_pwm_r_limiter = rt_SATURATE(tmp_pwm_r_limiter, -100.0F, 100.0F);

     pwm_r = tmp_pwm_r_limiter;

     /* Sum: '<S7>/Sum' incorporates:
      *  Gain: '<S7>/Gain'
      *  UnitDelay: '<S7>/Unit Delay'
      */
     tmp_pwm_l_limiter = (EXEC_PERIOD * tmp_thetadot_cmd_lpf) + ud_theta_ref;

     /* Sum: '<S10>/Sum' incorporates:
      *  Gain: '<S10>/Gain'
      *  UnitDelay: '<S10>/Unit Delay'
      */
     tmp_pwm_turn = (EXEC_PERIOD * tmp_psidot) + ud_psi;

     /* Sum: '<S5>/Sum' incorporates:
      *  Gain: '<S5>/Gain'
      *  Sum: '<S1>/Sum1'
      *  UnitDelay: '<S5>/Unit Delay'
      *  UnitDelay: '<S7>/Unit Delay'
      */
     tmp_pwm_r_limiter = ((ud_theta_ref - tmp_theta) * EXEC_PERIOD) + ud_err_theta;

     ud_err_theta = tmp_pwm_r_limiter;
     ud_theta_ref = tmp_pwm_l_limiter;
     ud_thetadot_cmd_lpf = tmp_thetadot_cmd_lpf;
     ud_psi = tmp_pwm_turn;
     ud_theta_lpf = tmp_theta_lpf;

}

void Segway::resetToZero()
{
    gyroOriginalTilts[0] = 0;
    gyroOriginalTilts[1] = 0;
    gyroOriginalTilts[2] = 0;
    gyroOffsetTilts[0] = 0.0;
    gyroOffsetTilts[1] = 0.0;
    gyroOffsetTilts[2] = 0.0;
    averageCount = 0;

    ud_err_theta = 0.0F;
    ud_theta_ref = 0.0F;
    ud_thetadot_cmd_lpf = 0.0F;
    ud_psi = 0.0F;
    ud_theta_lpf = 0.0F;

    args_battery = 0.0F;
    args_theta_m_l = 0.0F;
    args_theta_m_r = 0.0F;

    cmd_forward = 0;
    cmd_turn = 0;

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
    K_F[0] = list.at(0).toFloat();
    K_F[1] = list.at(1).toFloat();
    K_F[2] = list.at(2).toFloat();
    K_F[3] = list.at(3).toFloat();
    K_I = list.at(4).toFloat();
    qDebug("%f %f %f %f %f", K_F[0], K_F[1], K_F[2], K_F[3], K_I);
}
