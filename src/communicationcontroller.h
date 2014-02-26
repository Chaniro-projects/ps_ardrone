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
    enum eType {
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z
    };

    void sendTwistMsg(std::string to, geometry_msgs::Twist twist_msg);
    void sendTwistMsg(std::string to, float value, eType type);
    void sendTwistMsgRepeated(std::string to, geometry_msgs::Twist twist_msg, int refreshRate, float sec, bool verbose = false);
    void sendTwistMsgRepeated(std::string to, float value, eType type, int refreshRate, float sec, bool verbose = false);

    boost::thread* sendAsyncTwistMsg(std::string to, geometry_msgs::Twist twist_msg);
    boost::thread* sendAsyncTwistMsg(std::string to, float value, eType type);
    boost::thread* sendAsyncTwistMsgRepeated(std::string to, geometry_msgs::Twist twist_msg, int refreshRate, float sec, bool verbose = false);
    boost::thread* sendAsyncTwistMsgRepeated(std::string to, float value, eType type, int refreshRate, float sec, bool verbose = false);

private:
    //Singleton
    static CommunicationController* _cc;


};


#endif // COMMUNICATION_H
