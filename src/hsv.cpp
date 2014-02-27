#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

void tmp(int, void*) {}

int main(int argc, char** argv)
{
    cout << "Starting HSV node" << endl;
    ros::init(argc, argv,"hsv_node");

    string fileName;
    system("pwd");
    cout << "Fichier:" << endl;
    cin >> fileName;
    
    bool morph = false;
    
    CvCapture* capture = cvCaptureFromFile(fileName.c_str());
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    namedWindow("Trackbars", 0);
    IplImage* frame = NULL;
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, tmp);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, tmp);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, tmp);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, tmp);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, tmp);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, tmp);
    
    do {
        frame = cvQueryFrame(capture);
        
        if(frame != NULL) {
            Mat img(frame);
            resize(img, img, Size(800, 600));
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
    while(ros::ok() && frame != NULL);
    
    cvReleaseCapture(&capture);
    cvReleaseImage(&frame);

    return 0;
}
