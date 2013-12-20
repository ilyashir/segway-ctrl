#include "encodersReader.h"
#include "segway.h"
void EncodersReaderWorker::onTimerTick()
{
        emit resultReady(segway->readEncoders());
}
