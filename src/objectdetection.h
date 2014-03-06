#ifndef OBJECTDATA_H
#define OBJECTDATA_H

#include <iostream>
#include <pugixml/pugixml.hpp>
#include <fstream>
#include <vector>
#include <sstream>
#include "imageobject.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <string>


using namespace pugi;
using namespace cv;

class ObjectDetection
{
public:

    struct DetectionResult {
        bool detected;
        int centerX;
        int centerY;
        double size;
        float distance;
        cv::Mat* imgRange;
    };

    ObjectDetection();
    static ObjectDetection& getInstance();
    void showObjectsName();
    ImageObject* get(int i);
    ImageObject* get(std::string str);
    void save(std::string name, int hmin, int hmax, int smin, int smax, int vmin, int vmax);
    DetectionResult& detectObject(Mat& img, std::string objectName, bool morph = false, int minSize = 50, int resizeWidth = -1);
    void test();
    
private:
    std::ifstream *stream;
    xml_document *doc;
    static ObjectDetection* _od;
    std::vector<ImageObject> objs;
    int toInt(std::string str);
};

#endif // OBJECTDATA_H
