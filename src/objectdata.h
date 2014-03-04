#ifndef OBJECTDATA_H
#define OBJECTDATA_H

#include <iostream>
#include <pugixml/pugixml.hpp>
#include <fstream>
#include <vector>
#include "imageobject.h"
#include <sstream>

using namespace pugi;

class ObjectData
{
public:
    ObjectData();
    static ObjectData& getInstance();
    void showObjectsName();
    ImageObject* get(int i);
    ImageObject* get(std::string str);
    void save(std::string name, int hmin, int hmax, int smin, int smax, int vmin, int vmax);
    void test();
    
private:
    std::ifstream *stream;
    xml_document *doc;
    static ObjectData* _od;
    std::vector<ImageObject> objs;
    int toInt(std::string str);
};

#endif // OBJECTDATA_H
