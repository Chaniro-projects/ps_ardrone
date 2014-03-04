#ifndef IMAGEOBJECT_H
#define IMAGEOBJECT_H

#include <string>
#include <iostream>

class ImageObject
{
public:
    ImageObject();
    
    std::string name;
    int h_min;
    int h_max;
    int s_min;
    int s_max;
    int v_min;
    int v_max;
};

#endif // IMAGEOBJECT_H
