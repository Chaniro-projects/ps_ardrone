#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

class CameraController
{
public:
    CameraController();
    static CameraController& getInstance();
    void start();
    void stop();
    cv::Mat& getImage();
    void showLastImage();

private:
    //Singleton
    static CameraController* _cc;

    //Subcriber callback
    void image_callback(const sensor_msgs::ImageConstPtr& msg);

    //Save the last Image received
    cv::Mat lastImg;

    //Thread
    boost::thread *thread;
    void __start();
    bool started;
    boost::mutex img_mutex;
};

#endif // CAMERACONTROLLER_H
