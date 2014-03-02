#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void tmp(int, void*) {}

void hsvFromVideo();
void hsvFromImage();
void editObjFile();
void testDetectFromImg();

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

    cout << "Menu" << endl
         << "1) HSV from video" << endl
         << "2) HSV from image" << endl
         << "3) Detection test from image" << endl
         << "4) Edit obj file" << endl
         << "5) Exit" << endl;
    
    bool continu = true;
    int choix;
    cin >> choix;
    
    while(continu) {
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
            editObjFile();
            break;
        case 5:
            continu = false;
            break;
        }

    }

    return 0;
}

void testDetectFromImg() {
    string objFile, imgFile, line;
    bool morph = false;
    
    int H_min = 0,
        H_max = 256,
        S_min = 0,
        S_max = 256,
        V_min = 0,
        V_max = 256;
    
    cout << "Obj file:";
    cin >> objFile;
    cout << "Image:";
    cin >> imgFile;
    
    Mat img;
    img = imread(imgFile, CV_LOAD_IMAGE_COLOR);
    
    ifstream f(objFile.c_str());
            
    getline (f, line);
    getline (f, line);
    H_min = toInt(line);
    getline (f, line);
    H_max = toInt(line);
    getline (f, line);
    S_min = toInt(line);
    getline (f, line);
    S_max = toInt(line);
    getline (f, line);
    V_min = toInt(line);
    getline (f, line);
    V_max = toInt(line);
    
    resize(img, img, Size(600, img.rows*600/img.cols));
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
    RNG rng(12345);
    
    findContours(imgRange, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
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
        if(contours[i].size() > 15) {
            Scalar color = Scalar(255, 255, 255);
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
            circle( drawing, mc[i], 4, color, -1, 8, 0 );
            cout << "Shape center:(" << mc[i].x << "," << mc[i].y << ")    size:" << contours[i].size()  << endl;
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
            string name, fileName;
            cout << "Object name: ";
            cin >> name;
            
            ofstream file;
            fileName = name + ".obj";
            file.open(fileName.c_str());
            file << name << endl;
            file << H_min << endl;
            file << H_max << endl;
            file << S_min << endl;
            file << S_max << endl;
            file << V_min << endl;
            file << V_max << endl;
            file.close();
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
}