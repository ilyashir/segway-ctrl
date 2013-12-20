#include "stabilizer.h"
#include "segway.h"
void StabilizerWorker::onTimerTick() {
    segway->stabilization();
}
