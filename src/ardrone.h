#ifndef ARDRONE_H
#define ARDRONE_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "communicationcontroller.h"
#include "navdatacontroller.h"
#include "cameracontroller.h"
#include <boost/thread.hpp>

/*
 * ARDrone control class
 * Design pattern: singleton
 * Usage: Ardrone::getInstance().takeOff();
 * Method are both sync or async
 */

class Ardrone
{
public:
    Ardrone(int refresh, bool verbose);
    static Ardrone& getInstance(int refresh, bool verbose = false);

    //Sync
    void takeOff();
    void land();
    void reset();

    //Async
    void takeOffAsync();
    void landAsync();
    void resetAsync();

    void stopAll();

private:
    //Singleton
    static Ardrone* _ardrone;

    //Usefull stuff
    ros::NodeHandle node;
    ros::Rate loop_rate;
    int refreshRate;
    bool verbose;

    //Empty message
    std_msgs::Empty emp_msg;
    ros::Publisher pub_empty;

    //Thread function
    void __takeOff();
    void __land();
    void __reset();

    //Thread gestion
    boost::thread *lastThread;
    void stopAndWaitLastThread();
};

#endif // ARDRONE_H
