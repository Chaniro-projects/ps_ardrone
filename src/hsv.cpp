#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "objectdata.h"

using namespace std;
using namespace cv;

void tmp(int, void*) {}

void hsvFromVideo();
void hsvFromImage();
void editObjFile();
void testDetectFromImg();
void testDetectFromVideo();

int toInt(string str) {
    std::istringstream iss(str.c_str());
    int nombre;
    iss >> nombre;
    
    return nombre;
}

int main(int argc, char** argv)
{
    cout << "Starting HSV node" << endl;
    ros::init(argc, argv,"hsv_node");
    
    bool continu = true;
    int choix;
    
    
    while(continu) {
        cout << "Menu" << endl
             << "1) HSV from video" << endl
             << "2) HSV from image" << endl
             << "3) Detection test from image" << endl
             << "4) Detection test from video" << endl
             << "5) Edit obj file" << endl
             << "6) Show objects in objects.xml" << endl
             << "7) Exit" << endl
             << "92) Test" << endl;
        cin >> choix;
        switch(choix) {
        case 1:
            hsvFromVideo();
            break;
        case 2:
            hsvFromImage();
            break;
        case 3:
            testDetectFromImg();
            break;
        case 4:
            testDetectFromVideo();
            break;
        case 5:
            editObjFile();
            break;
        case 6:
            ObjectData::getInstance().showObjectsName();
            break;
        case 7:
            continu = false;
            break;
        case 92:
            ObjectData::getInstance().test();
            break;
        }
    }

    return 0;
}

void testDetectFromVideo() {
    string objName, imgFile, line;
    bool morph = false;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    cout << "Object name:";
    cin >> objName;
    cout << "Video:";
    cin >> imgFile;
    
    ImageObject* obj = ObjectData::getInstance().get(objName);
    
    std::cout << "h_min:" << obj->h_min << std::endl;
    
    H_min = obj->h_min;
    H_max = obj->h_max;
    S_min = obj->s_min;
    S_max = obj->s_max;
    V_min = obj->v_min;
    V_max = obj->v_max;
    
    CvCapture* capture = cvCaptureFromFile(imgFile.c_str());
    
    IplImage* frame = NULL;
    bool continu = true;
    
    do {
        frame = cvQueryFrame(capture);
        
        if(frame != NULL) {
            Mat img(frame);
            resize(img, img, Size(400, frame->height*400/frame->width));
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
            
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            Mat range;
            range = imgRange.clone();
            
            findContours(range, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));            
            
            //Moments   
            vector<Moments> mu(contours.size() );
            for( int i = 0; i < contours.size(); i++ )
            { if(contours[i].size() > 15) mu[i] = moments( contours[i], false ); }
            
            //Mass center
            vector<Point2f> mc( contours.size() );
              for( int i = 0; i < contours.size(); i++ )
                 { if(contours[i].size() > 15) mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
                    
            Mat drawing = Mat::zeros( imgRange.size(), CV_8UC3 );
            for( int i = 0; i< contours.size(); i++ ) {
                double size = contourArea(contours[i]);
                if(size > 50) {
                    Scalar color = Scalar(255, 255, 255);
                    drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
                    circle( drawing, mc[i], 2, color, -1, 8, 0 );
                    
                    ostringstream oss;
                    oss << "Center: (" << (int)(mc[i].x) << "," << (int)(mc[i].y) << ")   Size: " << (int)size;
                    ostringstream oss2;
                    oss2 << "Distance: " << (floor(obj->getDistance(size)*100)/100);
                    
                    putText(drawing, oss.str().c_str(), cv::Point(10,15), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
                    putText(drawing, oss2.str().c_str(), cv::Point(10,29), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
                }
            }
            
            imshow("Normal", img);
            imshow("HSV range", imgRange);
            imshow("Contour", drawing);
            
            
            int key = waitKey(25);
            
            if(key == 27)
                continu = false;
        }
    }
    while(frame != NULL && continu);
    
    cvReleaseCapture(&capture);
    cvReleaseImage(&frame);
    
    destroyAllWindows();
}

void testDetectFromImg() {
    string objName, imgFile;
    bool morph = false;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    cout << "Object name:";
    cin >> objName;
    
    ImageObject* obj = ObjectData::getInstance().get(objName);
    
    H_min = obj->h_min;
    H_max = obj->h_max;
    S_min = obj->s_min;
    S_max = obj->s_max;
    V_min = obj->v_min;
    V_max = obj->v_max;
    
    cout << "Image:";
    cin >> imgFile;
    
    Mat img;
    img = imread(imgFile, CV_LOAD_IMAGE_COLOR);
    
    resize(img, img, Size(600, img.rows*600/img.cols));
    Mat imgHsv, imgRange;
    
    cvtColor(img, imgHsv, COLOR_BGR2HSV);
    inRange(imgHsv, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), imgRange);
    
    int coefB = (obj->d2.size - (obj->d2.distance*obj->d1.size))/(obj->d1.distance - obj->d2.distance);
    int coefA = (obj->d2.size - coefB)/obj->d2.distance;
    
    if(morph) {
        Mat erodeElem = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElem = getStructuringElement(MORPH_RECT, Size(3, 3));
        
        erode(imgRange, imgRange, erodeElem);
        erode(imgRange, imgRange, erodeElem);
        
        dilate(imgRange, imgRange, dilateElem);
        dilate(imgRange, imgRange, dilateElem);
    }
  
    
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    Mat imgRange2 = imgRange.clone();
    
    findContours(imgRange2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    //Moments
    vector<Moments> mu(contours.size());
    for( int i = 0; i < contours.size(); i++ )
    { if(contours[i].size() > 15) mu[i] = moments( contours[i], false ); }
    
    //Mass center
    vector<Point2f> mc( contours.size() );
      for( int i = 0; i < contours.size(); i++ )
         { if(contours[i].size() > 15) mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
      
    Mat drawing = Mat::zeros( imgRange.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ) {
        double size = contourArea(contours[i]);
        if(size > 50) {
            Scalar color = Scalar(255, 255, 255);
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
            circle( drawing, mc[i], 4, color, -1, 8, 0 );
            ostringstream oss;
            oss << "Center: (" << (int)(mc[i].x) << "," << (int)(mc[i].y) << ")   Size: " << (int)size;
            ostringstream oss2;
            oss2 << "Distance: " << (floor((obj->getDistance(size))*100)/100);
            
            putText(drawing, oss.str().c_str(), cv::Point(10,15), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
            putText(drawing, oss2.str().c_str(), cv::Point(10,29), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
        }
    }
    
    
    imshow("Normal", img);
    imshow("HSV range", imgRange);
    imshow("Test", drawing);
    waitKey(0);
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
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, tmp);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, tmp);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, tmp);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, tmp);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, tmp);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, tmp);
    
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
            
            ObjectData::getInstance().save(name, H_min, H_max, S_min, S_max, V_min, V_max);
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
    
    createTrackbar("H_min", "Trackbars", &H_min, H_max, tmp);
    createTrackbar("H_max", "Trackbars", &H_max, H_max, tmp);
    createTrackbar("S_min", "Trackbars", &S_min, H_max, tmp);
    createTrackbar("S_max", "Trackbars", &S_max, H_max, tmp);
    createTrackbar("V_min", "Trackbars", &V_min, H_max, tmp);
    createTrackbar("V_max", "Trackbars", &V_max, H_max, tmp);
    
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
                
                ObjectData::getInstance().save(name, H_min, H_max, S_min, S_max, V_min, V_max);
                cout << "Object saved in '" << name << ".obj'";
            }
        }
    }
    while(ros::ok() && frame != NULL && go);
    
    cvReleaseCapture(&capture);
    cvReleaseImage(&frame);
}
