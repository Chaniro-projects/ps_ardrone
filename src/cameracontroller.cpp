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


void CameraController::findCircle(int minDist, int internalCanny, int centerDetection) {
    using namespace cv;
    Mat img = lastImg.clone();
    
    Mat cimg;
    cvtColor(img, cimg, COLOR_BGR2GRAY);
    
    //medianBlur(cimg, cimg, 5);
    //GaussianBlur(cimg, cimg, Size(9, 9), 2, 2);
    
    vector<Vec3f> circles;
    HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, minDist,
                 internalCanny, centerDetection, 5, 100);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( img, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, 0);
        circle( img, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, 0);
        std::cout << "Rayon: " << c[2] << std::endl;
    }

    cv::imshow("detected circles", img);
    cv::waitKey(0);
}

void CameraController::showImg(cv::Mat& img) {
    cv::imshow("detected circles", img);
    cv::waitKey(0);
}

void tmp(int, void*) {
    
}

void CameraController::threshHSV(bool morph) {
    using namespace cv;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    namedWindow("Trackbars", 0);
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, tmp);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, tmp);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, tmp);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, tmp);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, tmp);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, tmp);
    
    while(ros::ok()) {
        Mat img = lastImg.clone();
        Mat imgHsv, imgRange;
        
        cvtColor(img, imgHsv, COLOR_BGR2HSV);
        inRange(imgHsv, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), imgRange);
        
        if(morph) {
            Mat erodeElem = getStructuringElement(MORPH_RECT, Size(3, 3));
            Mat dilateElem = getStructuringElement(MORPH_RECT, Size(3, 3));
            
            erode(imgRange, imgRange, erodeElem);
            erode(imgRange, imgRange, erodeElem);
            
            dilate(imgRange, imgRange, dilateElem);
            dilate(imgRange, imgRange, dilateElem);
        }
        
        imshow("Normal", img);
        imshow("HSV", imgHsv);
        imshow("HSV range", imgRange);
        
        
        
        waitKey(30);
    }
}
