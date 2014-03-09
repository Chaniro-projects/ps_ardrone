#include "objectdetection.h"
ObjectDetection* ObjectDetection::_od = NULL;

ObjectDetection::ObjectDetection()
{
    stream = new std::ifstream(PATH_PKG"object/objects.xml");
    doc = new xml_document();
    doc->load(*(stream));
    
    for (xml_node obj = doc->child("objects").child("object"); obj; obj = obj.next_sibling("object"))
    {             
        ImageObject object;
        object.name = obj.child("name").child_value();
        object.h_min = toInt(obj.child("hmin").child_value());
        object.h_max = toInt(obj.child("hmax").child_value());
        object.s_min = toInt(obj.child("smin").child_value());
        object.s_max = toInt(obj.child("smax").child_value());
        object.v_min = toInt(obj.child("vmin").child_value());
        object.v_max = toInt(obj.child("vmax").child_value());
        
        ImageObject::Distance d1, d2;
        xml_node dist = obj.child("dist");
        d1.distance = dist.attribute("at").as_float();
        d1.size = dist.text().as_int();
        
        dist = dist.next_sibling("dist");
        d2.distance = dist.attribute("at").as_float();
        d2.size = dist.text().as_int();
        
        object.d1 = d1;
        object.d2 = d2;
        objs.push_back(object);
    }
}

ObjectDetection& ObjectDetection::getInstance() {
    if(ObjectDetection::_od == NULL)
        ObjectDetection::_od = new ObjectDetection();
    return *ObjectDetection::_od;
}

void ObjectDetection::showObjectsName() {
    for(int i = 0; i<objs.size(); i++)
        std::cout << std::endl << "(" << objs.at(i).name << ")" << std::endl;
}

ImageObject *ObjectDetection::get(int i) {
    if(i >= 0 && i < objs.size())
        return &objs.at(i);
    else
        return 0;
}

ImageObject *ObjectDetection::get(std::string str) {
    for(int i = 0; i<objs.size(); i++)
        if(str == objs.at(i).name)
            return &objs.at(i);
    return 0;
}

bool ObjectDetection::exist(std::string str)
{
    return get(str) != 0;
}

void ObjectDetection::save(std::string name, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    bool exist = false;
    for(int i = 0; i<objs.size(); i++)
        if(name == objs.at(i).name)
            exist = true;
    
    xml_node obj;
     
    if(exist) {
       
        for(obj = doc->child("objects").child("object"); obj; obj = obj.next_sibling())
            if(obj.child("name").child_value() == name)
                break;
        
        obj.child("hmin").text().set(hmin);
        obj.child("hmax").text().set(hmax);
        obj.child("smin").text().set(smin);
        obj.child("smax").text().set(smax);
        obj.child("vmin").text().set(vmin);
        obj.child("vmax").text().set(vmax);
    }
    else {
        obj = doc->child("objects").append_child("object");
        obj.append_child("name").text().set(name.c_str());
        obj.append_child("hmin").text().set(hmin);
        obj.append_child("hmax").text().set(hmax);
        obj.append_child("smin").text().set(smin);
        obj.append_child("smax").text().set(smax);
        obj.append_child("vmin").text().set(vmin);
        obj.append_child("vmax").text().set(vmax);
    }
    
    doc->save_file(PATH_PKG"object/objects.xml");
}

ObjectDetection::DetectionResult& ObjectDetection::detectObject(Mat &img, std::string objectName, bool morph, int minSize, int resizeWidth)
{
    DetectionResult res;
    res.detected = false;
    ImageObject* obj = get(objectName);
    
    if(resizeWidth > 0) 
        resize(img, img, Size(resizeWidth, img.rows*resizeWidth/img.cols));
    Mat imgHsv, imgRange;
    
    cvtColor(img, imgHsv, COLOR_BGR2HSV);
    inRange(imgHsv, Scalar(obj->h_min, obj->s_min, obj->v_min), Scalar(obj->h_max, obj->s_max, obj->v_max), imgRange);
    
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
      
    Mat* drawing = new Mat(Mat::zeros(imgRange.size(), CV_8UC3));
    for( int i = 0; i< contours.size(); i++ ) {
        double size = contourArea(contours[i]);
        if(size > minSize) {
            res.detected = true;
            res.centerX = (int)(mc[i].x);
            res.centerY = (int)(mc[i].y);
            res.distance = (floor((obj->getDistance(size))*100)/100);
            res.size = size;
            
            Scalar color = Scalar(255, 255, 255);
            drawContours( *drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
            circle( *drawing, mc[i], 4, color, -1, 8, 0 );
            std::ostringstream oss;
            oss << "Center: (" << res.centerX << "," << res.centerY << ")   Size: " << res.size;
            std::ostringstream oss2;
            oss2 << "Distance: " << res.distance;
            
            putText(*drawing, oss.str().c_str(), cv::Point(10,15), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
            putText(*drawing, oss2.str().c_str(), cv::Point(10,29), CV_FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255),1,8,false);
            res.imgRange = drawing;
            
            i = contours.size();
        }
    }
    
    return res;
}

void ObjectDetection::test()
{
    doc->child("objects").child("object").next_sibling("object").child("hmin").text().set(1992);
    doc->save_file("objects.xml");
}

int ObjectDetection::toInt(std::string str) {
    std::istringstream buffer(str);
    int v;
    buffer >> v;
    return v;
}
