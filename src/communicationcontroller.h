#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

/*
 * Communication control class (send message to other node)
 * Design pattern: singleton
 * Usage: CommunicationController::getInstance().sendAsyncEmptyMsg();
 * Method are both sync or async
 */

class CommunicationController
{
public:
    CommunicationController();
    static CommunicationController& getInstance();

    //empty
    void sendEmptyMsg(std::string to);
    void sendEmptyMsgRepeated(std::string to, int refreshRate, float sec, bool verbose = false);

    boost::thread* sendAsyncEmptyMsg(std::string to);
    boost::thread* sendAsyncEmptyMsgRepeated(std::string to, int refreshRate, float sec, bool verbose = false);


    //twist
    void sendTwistMsg(std::string to);
private:
    //Singleton
    static CommunicationController* _cc;


};

#endif // COMMUNICATION_H
