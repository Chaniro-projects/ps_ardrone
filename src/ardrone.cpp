#include "ardrone.h"

Ardrone* Ardrone::_ardrone = NULL;

Ardrone::Ardrone(int refresh, bool verbose) : loop_rate(refresh), refreshRate(refresh),
    verbose(verbose)
{}

Ardrone& Ardrone::getInstance(int refresh, bool verbose) {
    if(Ardrone::_ardrone == NULL)
        Ardrone::_ardrone = new Ardrone(refresh, verbose);
    return *Ardrone::_ardrone;
}

void Ardrone::takeOffAsync() {
    stopAndWaitLastThread();

    lastThread = CommunicationController::getInstance().sendAsyncEmptyMsgRepeated("/ardrone/takeoff", this->refreshRate, 5.0, verbose);
}

void Ardrone::takeOff() {
    stopAndWaitLastThread();

    CommunicationController::getInstance().sendEmptyMsgRepeated("/ardrone/takeoff", this->refreshRate, 5.0, verbose);

    lastThread = NULL;
}

void Ardrone::landAsync() {
    stopAndWaitLastThread();

    lastThread = CommunicationController::getInstance().sendAsyncEmptyMsgRepeated("/ardrone/land", this->refreshRate, 5.0, verbose);
}

void Ardrone::land() {
    stopAndWaitLastThread();

    CommunicationController::getInstance().sendEmptyMsgRepeated("/ardrone/land", this->refreshRate, 5.0, verbose);

    lastThread = NULL;
}

void Ardrone::resetAsync() {
    stopAndWaitLastThread();

    lastThread = CommunicationController::getInstance().sendAsyncEmptyMsgRepeated("/ardrone/reset", this->refreshRate, 3.0, verbose);
    system("rosservice  call /ardrone/setledanimation 1 5 2");
}

void Ardrone::reset() {
    stopAndWaitLastThread();

    CommunicationController::getInstance().sendEmptyMsgRepeated("/ardrone/reset", this->refreshRate, 3.0, verbose);
    system("rosservice  call /ardrone/setledanimation 1 5 2");

    lastThread = NULL;
}

void Ardrone::stopAndWaitLastThread() {
    if(lastThread != NULL) {
        lastThread->interrupt();
        lastThread->join();
    }
}

void Ardrone::stopAll() {
    stopAndWaitLastThread();;
}
