#include "communicationcontroller.h"
CommunicationController* CommunicationController::_cc = NULL;

CommunicationController::CommunicationController()
{
}

CommunicationController& CommunicationController::getInstance() {
    if(CommunicationController::_cc == NULL)
        CommunicationController::_cc = new CommunicationController();
    return *CommunicationController::_cc;
}

void CommunicationController::sendEmptyMsg(std::string to) {
    std_msgs::Empty emp_msg;
    ros::NodeHandle node;
    ros::Publisher pub_empty;

    pub_empty = node.advertise<std_msgs::Empty>(to, 1);
    pub_empty.publish(emp_msg);
}

void CommunicationController::sendEmptyMsgRepeated(std::string to, int refreshRate, float sec, bool verbose) {
    if(verbose)
        std::cout << "Sending to {" << to << "}" << std::endl;

    std_msgs::Empty emp_msg;
    ros::NodeHandle node;
    ros::Rate loop_rate(refreshRate);
    ros::Publisher pub_empty;

    pub_empty = node.advertise<std_msgs::Empty>(to, 1);
    pub_empty.publish(emp_msg);

    double time_start=(double)ros::Time::now().toSec();

    try {
        while (ros::ok()) {
            while ((double)ros::Time::now().toSec()< time_start+sec){
                boost::this_thread::interruption_point();
                pub_empty.publish(emp_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }
            if(verbose)
                std::cout << "Done {" << to << "}" << std::endl;
            break;
        }
    }
    catch(boost::thread_interrupted&) {
        if(verbose)
            std::cout << "Interupted after " << ((double)ros::Time::now().toSec() - time_start) << "s" << std::endl;
    }
}

boost::thread* CommunicationController::sendAsyncEmptyMsg(std::string to) {
    return new boost::thread(boost::bind(&CommunicationController::sendEmptyMsg, this, to));
}

boost::thread* CommunicationController::sendAsyncEmptyMsgRepeated(std::string to, int refreshRate, float sec, bool verbose) {
    return new boost::thread(boost::bind(&CommunicationController::sendEmptyMsgRepeated, this, to, refreshRate, sec, verbose));
}

void CommunicationController::sendTwistMsg(std::string to, geometry_msgs::Twist twist_msg) {
    ros::NodeHandle node;
    ros::Publisher pub;

    pub = node.advertise<geometry_msgs::Twist>(to, 1);
    pub.publish(twist_msg);
}

void CommunicationController::sendTwistMsgRepeated(std::string to, geometry_msgs::Twist twist_msg, int refreshRate, float sec, bool verbose) {
    if(verbose)
        std::cout << "Sending to {" << to << "}" << std::endl;

    ros::NodeHandle node;
    ros::Rate loop_rate(refreshRate);
    ros::Publisher pub;

    pub = node.advertise<geometry_msgs::Twist>(to, 1);
    pub.publish(twist_msg);

    double time_start=(double)ros::Time::now().toSec();

    try {
        while (ros::ok()) {
            while ((double)ros::Time::now().toSec()< time_start+sec){
                boost::this_thread::interruption_point();
                pub.publish(twist_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }
            if(verbose)
                std::cout << "Done {" << to << "}" << std::endl;
            break;
        }
    }
    catch(boost::thread_interrupted&) {
        if(verbose)
            std::cout << "Interupted after " << ((double)ros::Time::now().toSec() - time_start) << "s" << std::endl;
    }
}

boost::thread* CommunicationController::sendAsyncTwistMsg(std::string to, geometry_msgs::Twist twist_msg) {
    return new boost::thread(boost::bind(&CommunicationController::sendTwistMsg, this, to, twist_msg));
}

boost::thread* CommunicationController::sendAsyncTwistMsgRepeated(std::string to, geometry_msgs::Twist twist_msg, int refreshRate, float sec, bool verbose) {
    return new boost::thread(boost::bind(&CommunicationController::sendTwistMsgRepeated, this, to, twist_msg, refreshRate, sec, verbose));
}
