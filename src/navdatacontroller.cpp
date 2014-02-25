#include "navdatacontroller.h"
NavdataController* NavdataController::_nc = NULL;

NavdataController::NavdataController() : started(false)
{}

void NavdataController::start() {
    if(!started) {
        this->thread = new boost::thread(boost::bind(&NavdataController::__start, this));
        this->started = true;
    }
}

void NavdataController::__start() {
    ros::NodeHandle node;
    ros::Subscriber nav_sub;

    nav_sub = node.subscribe("/ardrone/navdata", 1, &NavdataController::navdata_callback, NavdataController::_nc);

    ros::Rate r(50);
    try {
        while(ros::ok()) {
            boost::this_thread::interruption_point();
            ros::spinOnce();
            r.sleep();
        }
    }
    catch(boost::thread_interrupted&) {}
}

void NavdataController::stop() {
    if(started) {
        this->thread->interrupt();
        this->thread->join();
    }
}

NavdataController& NavdataController::getInstance() {
    if(NavdataController::_nc == NULL)
        NavdataController::_nc = new NavdataController();

    return *NavdataController::_nc;
}

void NavdataController::navdata_callback(const ardrone_autonomy::Navdata &msg) {
    this->lastNavdata = &msg;
}

const ardrone_autonomy::Navdata& NavdataController::getNavdata() {
    return *(this->lastNavdata);
}
