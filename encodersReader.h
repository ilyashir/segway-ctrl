#pragma once

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QPair>
class Segway;

struct EncodersReaderWorker: public QObject
{
    Q_OBJECT
    QTimer timer;
    Segway * segway;
public:

    EncodersReaderWorker(Segway * _segway): segway(_segway){}

public slots:
    /// Initialize after correct thread is acquired
    void init() {
       connect(&timer, SIGNAL(timeout()), this, SLOT(onTimerTick()));
    }

    void start(int timespan) { timer.start(timespan); }
    void stop() { timer.stop(); }

    void onTimerTick();

signals:
    void resultReady(QPair<float, float>);
};


struct EncodersReader: public QObject {
    Q_OBJECT
public:
    EncodersReaderWorker worker;
    QThread  thread;

    EncodersReader(Segway * _segway):worker(_segway){}
    ~EncodersReader() { worker.stop(); thread.terminate(); thread.wait(100); }

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



