#include <ros/ros.h>
#include "ardrone.h"


int main(int argc, char** argv)
{
    using namespace std;
    cout << "Starting main node" << endl;
    ros::init(argc, argv,"main_node");

    int choix = 0;
    Ardrone ad = Ardrone::getInstance(50, true);
    
    NavdataController::getInstance().start();
    CameraController::getInstance().start();

    while(choix != -1 && ros::ok()) {
        cin >> choix;
        if(!ros::ok())
            break;
        switch(choix) {
        case 1:
            ad.takeOffAsync();
            break;
        case 2:
            ad.landAsync();
            break;
        case 3:
            ad.resetAsync();
            break;
        case 4:
            ad.takeOff();
            break;
        case 5:
            ad.land();
            break;
        case 6:
            ad.reset();
            break;
        case 7:
            cout << "Altitude: " << NavdataController::getInstance().getNavdata().altd << "mm" << endl;
            break;
        case 8:
            CameraController::getInstance().threshHSV(false);
            break;
        case 9:
            bool go = true;
            
            do {
                Mat img = CameraController::getInstance().getImage();
                ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, "green_circle", false, 50, 400);
                
                if(res.detected)
                    imshow("HSV range", *res.imgRange);
                
                imshow("Normal", img);
                
                int key = waitKey(25);
                if(key == 27)
                  go = false;
                
                delete res.imgRange;
            }
            while(ros::ok() && go);
            break;
        }
    }

    return 0;
}

