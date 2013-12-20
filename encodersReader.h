#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include <QObject>

class WorkerThread : public QObject
{
    Q_OBJECT
public:
    explicit WorkerThread(QObject *parent = 0);

signals:

public slots:

};

#endif // WORKERTHREAD_H
