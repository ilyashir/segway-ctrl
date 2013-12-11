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
#include <QTime>
#include <sys/time.h>
#include <time.h>

#define CMD_MAX                 100.0F
#define DEG2RAD                 0.01745329238F
#define GYRO_TH                 0.00052F
#define rt_SATURATE(sig,ll,ul)  (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )

const float FLOAT_ACCURACY = 1.0e-7;
const float A_R = 0.996F; /* low pass filter gain for motors target count */
const float A_D = 0.8F; /* low pass filter gain for motors average count */
const float gyroConst = /*938.736454707F; */ 820.846931; /*Parrots to rad*/
const float accConst_DO_NOT_USE = 0.000244140625; /*Parrots to m/sec2 */
const float K_THETADOT = 6.25F;  /* 0.3/R*/
const float K_PHIDOT = 25.0F;  /* turn target speed gain */
const float BATTERY_GAIN = 0.024482409F;  // 0.018504035F; //old	/* battery voltage gain for motor PWM outputs */
const float BATTERY_OFFSET = 31.96455651F;	/* battery voltage offset for motor PWM outputs */
const float COMPL_K = 0.02;
QTime GTime;
float EXEC_PERIOD=0;
float ugol = 0;

template <typename T> 
inline const QVector3D fromQVector(const QVector<T> & src) 
{
    // hope, vector has exactly 3 elements 
    return QVector3D(src[0], src[1], src[2]);
}

Segway::Segway(QThread *guiThread) :
    brick(*guiThread),
    segwayState(INIT_MODE),
    averageCount(0)
{
 /* 
    K_F[0] = -93.813117253222; 
    K_F[1] = -1409.752452517953; 
    K_F[2] = -44.386511747935;
    K_F[3] = -94.233089049743; 
    K_I = 0; //-99.999999914505; 
    KK_I = 1400;

    K_F[0] = -14.38042459;
    K_F[1] = -278.8513314;
    K_F[2] = -7.932820031;
    K_F[3] = -20.34566572;
    K_I =    34;
*/
/*
    K_F[0] = -33.9211; // -35.9211; //-14.38042459;
    K_F[1] = -550.860; // -278.8513314;
    K_F[2] = -12.0395; // -14.0395; //-7.932820031;
    K_F[3] = -36.0243; // -26.0243; // -20.34566572;
    K_I =    -10.7298; //34
*/
/*
    K_F[0] = -28.17935213753;
    K_F[1] = -2478.605166776197; //1878
    K_F[2] = -28.439276418593;
    K_F[3] = -169.592478101189; //99
    K_I =    -0; 
*/
    K_F[0] = -13.4532188075615; //13
    K_F[1] = -1453.3180453519147; //853
    K_F[2] = -14.6826705718868; // 14
    K_F[3] = -55.1241703490949; //50
    K_I =    -7.3245553202977; 
    KK_I = 0;

    resetToZero();

    qDebug() << "INIT_MODE";

    qDebug("%f %f %f %f %f", K_F[0], K_F[1], K_F[2], K_F[3], K_I);
    qDebug() << "ud_psi: " << ud_psi;

    GTime.start();
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
          brick.encoder("3")->reset(); //left encoder
          brick.encoder("4")->reset(); //right encoder

          segwayState = CALC_MODE;
          qDebug() << "CALC_MODE";

          QTimer::singleShot(2000, this, SLOT(startStabilization()));

          qDebug() << "ud_psi: " << ud_psi;

          connect(&taskTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
          taskTimer.start(4);
          break;
      case CALC_MODE:
      case CONTROL_MODE:
          segwayState = INIT_MODE;
	        resetToZero();
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
    gyroOriginalTilts += fromQVector(brick.gyroscope()->read());
    accOriginal += fromQVector(brick.accelerometer()->read());
    ++averageCount;
}

// накап
void Segway::getVoltage()
{
    float voltage = brick.battery()->readVoltage();
    args_battery = voltage * 1000; //mV
}

void Segway::startStabilization()
{
    taskTimer.stop();
    

    gyroOffsetTilts = gyroOriginalTilts / averageCount;
    accOriginal.normalize();    
    
    QMatrix3x3 X_,XT,X;
    X_(0,0) = accOriginal.length(); // 1 ???
    
    XT(0,0) = accOriginal.x();
    XT(0,1) = accOriginal.y();
    XT(0,2) = accOriginal.z();

    X = XT.transposed();

    QMatrix4x4 temp1(X_*XT);
    
    bool inverted;
    QMatrix4x4 temp2 = QMatrix4x4(X*XT).inverted(&inverted);
        
    accRotate = temp1 * temp2;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
      {
        qreal & cell = accRotate(i,j);
        if (-FLOAT_ACCURACY < cell && cell < FLOAT_ACCURACY)
          cell = 0;
      }
    
 
    args_battery = brick.battery()->readVoltage() * 1000;
//    qDebug() << "arg_bat:" << args_battery;

    segwayState = CONTROL_MODE;
    qDebug() << "CONTROL_MODE";

    
    disconnect(&taskTimer, SIGNAL(timeout()), this, SLOT(prepareSegway()));
    connect(&taskTimer, SIGNAL(timeout()), this, SLOT(stabilization()));

    batteryTimer.start(20);
    taskTimer.start(4);
}

void Segway::stabilization()
{

#if 1
    static struct timespec last_start_at;
    struct timespec begin_at;
    clock_gettime(CLOCK_MONOTONIC, &begin_at);
#endif

/*
    (void)ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
    * R/C command 
    * buf[0]: -100(forward max.) to 100(backward max.)
    * buf[1]: -100(turn left max.) to 100(turn right max.)
    */
    cmd_forward = 0;
    cmd_turn = 0;
    /*
    cmd_forward = -(S8)bt_receive_buf[0];  reverse the direction
    cmd_turn = (S8)bt_receive_buf[1];
    */
    
    gyroOriginalTilts = fromQVector(brick.gyroscope()->read());

    QVector3D temp  = accRotate * fromQVector(brick.accelerometer()->read());
    acc = acos (temp.x() / temp.length());

    qDebug() << "acc: " << acc; 

#if 1
    struct timespec phase1_at;
    clock_gettime(CLOCK_MONOTONIC, &phase1_at);
#endif

    args_theta_m_l = - brick.encoder("3")->read();
    args_theta_m_r = brick.encoder("4")->read();

#if 1
    struct timespec phase2_at;
    clock_gettime(CLOCK_MONOTONIC, &phase2_at);
#endif

//    qDebug("left encoder: %f right encoder: %f", args_theta_m_l, args_theta_m_r);

    balance_control();

    int p_l = (int)pwm_l;
    int p_r = (int)pwm_r;

   qDebug("pwmL: %d pwmR: %d", p_l, p_r);
//    qDebug() << "--------------------";

#if 1
    struct timespec phase3_at;
    clock_gettime(CLOCK_MONOTONIC, &phase3_at);
#endif

    brick.powerMotor("3")->setPower(p_l);
    brick.powerMotor("4")->setPower(p_r);

#if 0
    struct timespec end_at;
    clock_gettime(CLOCK_MONOTONIC, &end_at);
    qDebug("interval %ldns, elapsed total %ld, gyro %ld, enc %ld, balance %ld, motor %ld ns", 
           begin_at.tv_nsec-last_start_at.tv_nsec,
           end_at.tv_nsec-begin_at.tv_nsec,
           phase1_at.tv_nsec-begin_at.tv_nsec, phase2_at.tv_nsec-phase1_at.tv_nsec, phase3_at.tv_nsec-phase2_at.tv_nsec, end_at.tv_nsec-phase3_at.tv_nsec);
    last_start_at.tv_nsec = begin_at.tv_nsec;
#endif
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

    static struct timespec last_ts;
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const long long diffNs = ((long long)now_ts.tv_nsec - (long long)last_ts.tv_nsec) + ((long long)now_ts.tv_sec - (long long)last_ts.tv_sec)*1000000000ll;
    last_ts = now_ts;
    if (last_ts.tv_sec != 0)
        EXEC_PERIOD = (float)diffNs/1000000000.0f;

    qDebug() << "EXEC_PERIOD" << EXEC_PERIOD;
    
    // учет управления с пульта: скорость на колеса (фильтр)
    tmp_thetadot_cmd_lpf = (((cmd_forward / CMD_MAX) * K_THETADOT) * (1.0F - A_R)) + (A_R * ud_thetadot_cmd_lpf);
//    qDebug() << "движение tmp_thetadot_cmd_lpf=" << tmp_thetadot_cmd_lpf; 

     // угол с энкодеров (от разницы поворота колес за прошедшее время), радианы
    tmp_theta = (args_theta_m_l - args_theta_m_r) * 0.5F + ud_psi;
 //    qDebug() << "       tmp_theta=" << tmp_theta; 

    // фильтрация угла отклонения (от разницы поворота колес)
    tmp_theta_lpf = ((1.0F - A_D) * tmp_theta) + (A_D * ud_theta_lpf);
 //    qDebug() << "filter tmp_theta=" << tmp_theta_lpf;

    // угловая скорость отклонения по гироскопу (рад за прошедшее время)
    tmp_psidot = (gyroOriginalTilts.x()  - gyroOffsetTilts.x()) / gyroConst;

//    if (abs(tmp_psidot)<GYRO_TH*EXEC_PERIOD) tmp_psidot=0;

//    qDebug() << "Gyro " << gyro*180 / M_PI ;
//    qDebug() << "gyro считанное  (гр):" << gyroOriginalTilts[0] * 180 / (gyroConst*M_PI);
//    qDebug() << "gyro отклонение (гр):" << tmp_psidot*180 / M_PI << " (rad)" << tmp_psidot;
 
    tmp[0] = ud_theta_ref;		// накопленный угол при управлении с пульта
    tmp[1] = 0.0F;			
    tmp[2] = tmp_thetadot_cmd_lpf;	// управление с пульта 
    tmp[3] = 0.0F;
    tmp_theta_0[0] = tmp_theta;		// угол 
    tmp_theta_0[1] = ud_psi;		// 
    qDebug() << "ud_psi(deg) = " << ud_psi*180/M_PI;
    tmp_theta_0[2] = (tmp_theta_lpf - ud_theta_lpf) / EXEC_PERIOD;	//скорость моторов (рад/c)
    tmp_theta_0[3] = tmp_psidot;
    tmp_pwm_r_limiter = 0.0F;

    for (int i = 0; i < 4; i++) {
        tmp_pwm_r_limiter += (tmp[i] - tmp_theta_0[i]) * K_F[i];
//       qDebug("%f, %f ", tmp[i] - tmp_theta_0[i], (tmp[i] - tmp_theta_0[i]) * K_F[i]);
    }
//    qDebug("%f, %f ", tmp[0] - tmp_theta_0[0], (tmp[0] - tmp_theta_0[0]) * K_F[0]);

//    qDebug() << "pwm_r (без KI&батареи): " << tmp_pwm_r_limiter;

//    qDebug("%f, %f ", ud_err_theta, K_I * ud_err_theta);
    ud_err_psi = ud_err_psi + ud_psi * EXEC_PERIOD;
//    Debug() << "ud_err_psi: " << KK_I * ud_err_psi;

    tmp_pwm_r_limiter = (((K_I * ud_err_theta) + (KK_I * ud_err_psi) + tmp_pwm_r_limiter) / ((BATTERY_GAIN * args_battery) - BATTERY_OFFSET)) * 100.0F;
//    qDebug() << "pwm_r (без батареи):" << ((K_I * ud_err_theta) + tmp_pwm_r_limiter) << " BT: " << ((BATTERY_GAIN * args_battery) - BATTERY_OFFSET) << " args_bat:" << args_battery;
//    qDebug() << "pwm_r_limiter: " << tmp_pwm_r_limiter;

    tmp_pwm_turn = (cmd_turn / CMD_MAX) * K_PHIDOT;

    tmp_pwm_l_limiter = tmp_pwm_r_limiter + tmp_pwm_turn;

    tmp_pwm_l_limiter = rt_SATURATE(tmp_pwm_l_limiter, -100.0F, 100.0F);

    pwm_l = 0; //- tmp_pwm_l_limiter;

    tmp_pwm_r_limiter -= tmp_pwm_turn;

    tmp_pwm_r_limiter = rt_SATURATE(tmp_pwm_r_limiter, -100.0F, 100.0F);

//    qDebug() << "pwm_r_limiter: " << tmp_pwm_r_limiter << "pwm_l_limiter: " << tmp_pwm_l_limiter;

    pwm_r = 0; //- tmp_pwm_r_limiter;

    ud_err_theta = ((ud_theta_ref - tmp_theta) * EXEC_PERIOD) + ud_err_theta; //новая ошибка энкодеров
//    qDebug() << "ud_theta_ref:" << ud_theta_ref << "ud_err_theta" << ud_err_theta;

    ud_theta_ref = (EXEC_PERIOD * tmp_thetadot_cmd_lpf) + ud_theta_ref;

//    ugol = (1 - COMPL_K) * ((EXEC_PERIOD * tmp_psidot) + ugol) + COMPL_K * atan(-(float)acc.z()/(float)acc.x());
    ud_psi = (1 - COMPL_K) * ((EXEC_PERIOD * tmp_psidot) + ud_psi) + COMPL_K * acc; // новый угол с гироскопа
//    qDebug() << "ugol: " << ugol*180/M_PI;

    ud_thetadot_cmd_lpf = tmp_thetadot_cmd_lpf; //запоминаем фильтрованное значение с пульта для следующего шага
 
    ud_theta_lpf = tmp_theta_lpf; //запоминаем угол поворота колес для следующего шага

}

void Segway::resetToZero()
{
    gyroOriginalTilts = QVector3D();
    gyroOffsetTilts = QVector3D();
    acc = 0;
    alpha_acc = QVector3D();
    averageCount = 0;

    ud_err_psi = 0.0F;
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
