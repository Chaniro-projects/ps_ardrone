#include <ros/ros.h>
#include "ardrone.h"


int main(int argc, char** argv)
{
    using namespace std;
    cout << "Starting main node" << endl;
    ros::init(argc, argv,"main_node");

    int choix = 0;
    Ardrone ad = Ardrone::getInstance(200, true);
    
    NavdataController::getInstance().start();
    CameraController::getInstance().start(); 

    //float speed = 0.2;
    float speed = 0.1;
    double a, b;
    ros::Rate loop_rate(200);
    
    geometry_msgs::Twist twist_msg;
    bool go = true;
    double step = 0;
    
    while(choix != -1 && ros::ok()) {
        cin >> choix;
        if(!ros::ok())
            break;
        switch(choix) {
        case 4:
            ad.takeOff();
            break;
        case 5:
            ad.land();
            break;
        case 6:
            CommunicationController::getInstance().sendAsyncTwistMsg("/cmd_vel", 0, CommunicationController::linear_x);
            ad.reset();
            break;
        case 7:
            std::cout << "takeOff" << std::endl;
            ad.takeOff();
            
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 1);
            
            std::cout << "Starting detection" << std::endl;
            
            step = (double)ros::Time::now().toSec();

            std::cout << "droit" << std::endl;
            while (ros::ok() && ros::Time::now().toSec() < step + 3) {
                CommunicationController::getInstance().sendTwistMsg("/cmd_vel", -speed, CommunicationController::linear_y);
            }
            std::cout << "stop" << std::endl;
            while (ros::ok() && ros::Time::now().toSec() < step + 6) {
                CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_y);
            }
            std::cout << "gauche" << std::endl;
            while (ros::ok() && ros::Time::now().toSec() < step + 9) {
                CommunicationController::getInstance().sendTwistMsg("/cmd_vel", speed, CommunicationController::linear_y);
            }
            
            std::cout << "Landing" << std::endl;
            ad.land();
            
            break;
        case 8:
            std::cout << "takeOff" << std::endl;
            ad.takeOff();
            
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 1);
            
            
            std::cout << "Starting detection" << std::endl;
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 15) {
                a = (double)ros::Time::now().toSec();
                Mat img(CameraController::getInstance().getImage());
                ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, "rose", false, 50, 400);
                            
                if(res.detected) {
                    
                    //forward
                    if(res.distance < 1.2) {
                        twist_msg.linear.x = -speed;
                    }else if(res.distance >= 1.9){
                        twist_msg.linear.x = speed;
                    }
                    
                    //320x240
                    //right/left
                    if(res.centerX > (200)) {
                        twist_msg.linear.y = -2 * speed;
                    }else if(res.centerX < (120)){
                        twist_msg.linear.y = 2 * speed;
                    }
                    CommunicationController::getInstance().sendTwistMsg("/cmd_vel", twist_msg);
                    twist_msg.linear.x = 0;
                    twist_msg.linear.y = 0;
                }
                else {
                    CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_x);        
                }
                
                
                ros::spinOnce();
                loop_rate.sleep();
                b = (double)ros::Time::now().toSec();
                std::cout << (b-a) << std::endl;
            }
            
            
            std::cout << "Landing" << std::endl;
            ad.land();
            
            break;
        case 9:
            std::cout << "takeOff..." << std::endl;
            ad.takeOff();
            std::cout << "Done TakeOff" << std::endl;
            
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 1);
            
            step = (double)ros::Time::now().toSec();
            speed = 0.1;
            while (ros::ok() && ros::Time::now().toSec() < step + 15) {
                Mat img(CameraController::getInstance().getImage());
                ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, "rose", false, 50, 400);
                
                if(res.detected) {
                    if(res.centerX > (213)) {
                        std::cout << "Detected: droite " << res.centerX << std::endl;
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", -speed, CommunicationController::linear_z);
                    }else if(res.centerX < (107)){
                        std::cout << "Detected: gauche " << res.centerX << std::endl;
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", speed, CommunicationController::linear_z);
                    }else{
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_z);
                        std::cout << "Detected: rien " << res.centerX << std::endl;
                    }
                }
                else {
                    CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_z);
                    std::cout << "No detected " << std::endl;           
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            
           
            
            std::cout << "Landing..." << std::endl;
            ad.land();
            std::cout << "Done" << std::endl;
            
            break;
            
        case 1:
            
            std::cout << "takeOff..." << std::endl;
            ad.takeOff();
            std::cout << "Done TakeOsff" << std::endl;
            
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 1);
            /*ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, "bowl", false, 50, 400);
            
            if(res.detected)
                std::cout << "Detected on:  " << res.centerX << ":" << res.centerY << " at " << res.distance << std::endl;
            else
                std::cout << "." << std::endl;*/
            
            step = (double)ros::Time::now().toSec();
            while (ros::ok() && ros::Time::now().toSec() < step + 15) {
                Mat img(CameraController::getInstance().getImage());
                ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, "rose", false, 50, 400);
                            
                if(res.detected) {
                    if(res.distance < 1.2) {
                        std::cout << "Detected: recule " << res.distance << std::endl;
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", -speed, CommunicationController::linear_x);
                    }else if(res.distance >= 1.9){
                        float tmp = speed * (res.distance - 1.9);
                        if(tmp > speed*1.5)
                            tmp = speed;
                        std::cout << "Detected: avance " << res.distance << std::endl;
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", tmp, CommunicationController::linear_x);
                    }else{
                        CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_x);
                        std::cout << "Detected: rien " << res.distance << std::endl;
                    }
                }
                else {
                    CommunicationController::getInstance().sendTwistMsg("/cmd_vel", 0, CommunicationController::linear_x);
                    std::cout << "No detected " << std::endl;           
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            
           
            
            std::cout << "Landing..." << std::endl;
            ad.land();
            std::cout << "Done" << std::endl;
            
            break;
        }
    }

    return 0;
}

