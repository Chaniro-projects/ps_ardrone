#ifndef NAVDATACONTROLLER_H
#define NAVDATACONTROLLER_H

#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include <boost/thread.hpp>

/*
 * Navdata control class (get navigation information)
 * Design pattern: singleton
 * Usage: NavdataController::getInstance().getNavdata();
 * Call start to start subscribe to /ardrone/navdata topic
 *  then call stop to end it
 */

class NavdataController
{
public:
    NavdataController();
    static NavdataController& getInstance();
    void start();
    void stop();
    const ardrone_autonomy::Navdata& getNavdata();

private:
    //Singleton
    static NavdataController* _nc;

    //Subcriber callback
    void navdata_callback(const ardrone_autonomy::Navdata &msg);

    //Save the last Navdata received
    const ardrone_autonomy::Navdata* lastNavdata;

    //Thread
    boost::thread *thread;
    void __start();
    bool started;
};

#endif // NAVDATACONTROLLER_H
