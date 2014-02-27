#include "cameracontroller.h"
CameraController* CameraController::_cc = NULL;

CameraController::CameraController() : started(false)
{
}

CameraController& CameraController::getInstance() {
    if(CameraController::_cc == NULL)
        CameraController::_cc = new CameraController();
    return *CameraController::_cc;
}

void CameraController::start() {
    if(!started) {
        this->thread = new boost::thread(boost::bind(&CameraController::__start, this));
        this->started = true;
    }
}

void CameraController::__start() {
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);

    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, &CameraController::image_callback, CameraController::_cc);

    ros::Rate r(50);
    try {
        while(ros::ok()) {
            boost::this_thread::interruption_point();
            ros::spinOnce();
            r.sleep();
        }
    }
    catch(boost::thread_interrupted&) {
        sub.shutdown();
    }
}

void CameraController::stop() {
    if(started) {
        this->thread->interrupt();
        this->thread->join();
    }
}


void CameraController::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    img_mutex.lock();
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        this->lastImg = cv_ptr->image;
    }
    catch(cv_bridge::Exception& e) {
        std::cout << "Erreur cv_bridge -> " << e.what() << std::endl;
    }

    img_mutex.unlock();
}

cv::Mat& CameraController::getImage() {
    return (this->lastImg);
}

void CameraController::showLastImage() {

    img_mutex.lock();

    cv::imshow("Camera", this->lastImg);
    cv::waitKey(1);

    img_mutex.unlock();
}


void CameraController::perform() {
    using namespace cv;
    Mat cimg;
    medianBlur(lastImg, lastImg, 5);
    cvtColor(lastImg, cimg, COLOR_RGB2GRAY);
    std::cout << lastImg.depth() << "   " << cimg.depth() << std::endl;
    imshow("detected circles", cimg);
    waitKey();
    
    /*GaussianBlur(cimg, cimg, Size(9, 9), 2, 2);

    imshow("detected circles", cimg);
    waitKey();*/
    
    vector<Vec3f> circles;
    HoughCircles(lastImg, circles, CV_HOUGH_GRADIENT, 1, 10,
                 100, 30, 1, 30 // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( cimg, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, 0);
        circle( cimg, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, 0);
    }

    imshow("detected circles", cimg);
    waitKey();
}
