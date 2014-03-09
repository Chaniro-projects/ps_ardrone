#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "objectdetection.h"
#include "cameracontroller.h"
//#include "detectiontool.h"

using namespace std;
using namespace cv;

void hsvFromVideo();
void hsvFromImage();
void hsvFromDrone();
void editObjFile();
void testDetectFromImg();
void testDetectFromVideo();
void testDetectFromDrone();

int toInt(string str) {
    std::istringstream iss(str.c_str());
    int nombre;
    iss >> nombre;
    
    return nombre;
}

int main(int argc, char** argv)
{
    /*QApplication app(argc, argv);
    DetectionTool dt(&app);
    return dt.go();*/
    
    ros::init(argc, argv,"hsv_node");
    
    CameraController::getInstance().start();
    
    cout << "Starting HSV node" << endl;
    ros::init(argc, argv,"hsv_node");
    
    bool continu = true;
    int choix;
    
    
    while(continu) {
        cout << "Menu" << endl
             << "1) HSV from video" << endl
             << "2) HSV from image" << endl
             << "3) HSV from AR.Drone camera" << endl
             << "4) Detection test from image" << endl
             << "5) Detection test from video" << endl
             << "6) Detection test from AR.Drone" << endl
             << "7) Show objects in objects.xml" << endl
             << "8) Exit" << endl
             << "42) Test" << endl;
        cin >> choix;
        switch(choix) {
        case 1:
            hsvFromVideo();
            break;
        case 2:
            hsvFromImage();
            break;
        case 3:
            hsvFromDrone();
            break;
        case 4:
            testDetectFromImg();
            break;
        case 5:
            testDetectFromVideo();
            break;
        case 6:
            testDetectFromDrone();
            break;
        case 7:
            ObjectDetection::getInstance().showObjectsName();
            break;
        case 8:
            continu = false;
            break;
        case 42:
            
            break;
        }
    }

    return 0;
}

void testDetectFromDrone() {
    string objName;
    
    cout << "Object name:";
    cin >> objName;
    
    bool continu = true;
    
    do {
        Mat img(CameraController::getInstance().getImage());
        Mat black(Mat::zeros(400, img.rows*400/img.cols, CV_8UC3));
        
        ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, objName, false, 50, 400);
        
        if(res.detected)
            imshow("Contour", *res.imgRange);
        else
            imshow("Contour", black);
        
        imshow("Normal", img);
        int key = waitKey(25);
        if(key == 27)
            continu = false;
        
        if(res.detected)
            delete res.imgRange;
    }
    while(continu);
    
    destroyWindow("Contour");
    destroyWindow("Normal");
}

void hsvFromDrone() {
    //CameraController::getInstance().start();
    
    bool morph = false;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    namedWindow("Trackbars", 0);
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, NULL);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, NULL);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, NULL);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, NULL);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, NULL);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, NULL);
    
    bool go = true;
    std::cout << "go" << std::endl;
    do {
        Mat img(CameraController::getInstance().getImage());
        resize(img, img, Size(400, img.rows*400/img.cols));
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
        imshow("HSV range", imgRange);
        
        int key = waitKey(30);
        if(key == 27)
          go = false;
        else if (key == 115) {
            destroyAllWindows();
            string name;
            cout << "Object name:";
            cin >> name;
            
            ObjectDetection::getInstance().save(name, H_min, H_max, S_min, S_max, V_min, V_max);
            cout << "Object saved in '" << name << ".obj'";
            
        }
    }
    while(ros::ok() && go);
}

void testDetectFromVideo() {
    string objName, imgFile;
    
    cout << "Object name:";
    cin >> objName;
    cout << "Video:";
    cin >> imgFile;
    
    CvCapture* capture = cvCaptureFromFile(imgFile.c_str());
    
    IplImage* frame = NULL;
    bool continu = true;
    
    do {
        frame = cvQueryFrame(capture);
        
        if(frame != NULL) {
            Mat img(frame);
            
            ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, objName, false, 50, 400);
            
            if(res.detected) {
                imshow("Contour", *res.imgRange);
            }
            
            imshow("Normal", img);
            int key = waitKey(25);
            if(key == 27)
                continu = false;
            
            delete res.imgRange;
        }
    }
    while(frame != NULL && continu);
    
    destroyWindow("Contour");
    destroyWindow("Normal");
    
    cvReleaseCapture(&capture);
    cvReleaseImage(&frame);
}

void testDetectFromImg() {
    string objName, imgFile;
    
    cout << "Object name:";
    cin >> objName;
    
    cout << "Image:";
    cin >> imgFile;
    
    Mat img = imread(imgFile);
    
    ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, objName);
    
    if(res.detected) {
        imshow("Normal", img);
        imshow("Test", *res.imgRange);
        waitKey(0);
    }
    else
        std::cout << "Object not found on image '" << imgFile << "'.";
    
    delete res.imgRange;
}

void editObjFile() {
    
}

void hsvFromImage() {
    string fileName;
    cout << "Fichier:" << endl;
    cin >> fileName;
    
    Mat source;
    source = imread(fileName, CV_LOAD_IMAGE_COLOR);
    
    bool morph = false;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    namedWindow("Trackbars", 0);
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, NULL);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, NULL);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, NULL);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, NULL);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, NULL);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, NULL);
    
    bool go = true;
    
    do {
       
        Mat img = source.clone();
        
        resize(img, img, Size(600, source.rows*600/source.cols));
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
        imshow("HSV range", imgRange);
        
        
        int key = waitKey(30);
        if(key == 27)
          go = false;
        else if (key == 115) {
            destroyAllWindows();
            string name;
            cout << "Object name:";
            cin >> name;
            
            ObjectDetection::getInstance().save(name, H_min, H_max, S_min, S_max, V_min, V_max);
            cout << "Object saved in '" << name << ".obj'";
        }
        
    }
    while(ros::ok() && go);
}

void hsvFromVideo() {
    string fileName;
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
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, NULL);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, NULL);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, NULL);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, NULL);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, NULL);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, NULL);
    
    bool go = true;
    
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
            
            
            
            int key = waitKey(30);
            if(key == 27)
              go = false;
            else if (key == 115) {
                destroyAllWindows();
                string name;
                cout << "Object name:";
                cin >> name;
                
                ObjectDetection::getInstance().save(name, H_min, H_max, S_min, S_max, V_min, V_max);
                cout << "Object saved in '" << name << ".obj'";
            }
        }
    }
    while(ros::ok() && frame != NULL && go);
    
    cvReleaseCapture(&capture);
    cvReleaseImage(&frame);
}


