#pragma once

#include <QObject>
#include <QTimer>
#include <QThread>

class Segway;

struct StabilizerWorker: public QObject
{
    Q_OBJECT
    QTimer timer;
    Segway * segway;
public:

    StabilizerWorker(Segway * _segway): segway(_segway){}

public slots:
    /// Initialize after correct thread is acquired
    void init() {
       connect(&timer, SIGNAL(timeout()), this, SLOT(onTimerTick()));
    }

    void start(int timespan) { timer.start(timespan); }
    void stop() { timer.stop(); }

    void onTimerTick();

signals:
    void resultReady();
};


struct Stabilizer: public QObject {
    Q_OBJECT
public:
    StabilizerWorker worker;
    QThread  thread;
    Stabilizer(Segway * _segway):worker(_segway){}
    ~Stabilizer() { worker.stop(); thread.terminate(); thread.wait(100); }

    void init() {
        worker.moveToThread(&thread);
        connect(&thread, SIGNAL(started()), &worker, SLOT(init()));
        connect(&thread, SIGNAL(finished()), &worker, SLOT(deleteLater()));
    }

    void start(int timespan) {
        thread.start();
        worker.start(timespan);
    }

    void stop() { worker.stop();}

};



