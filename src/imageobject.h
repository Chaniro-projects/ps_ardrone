#ifndef IMAGEOBJECT_H
#define IMAGEOBJECT_H

#include <string>

class ImageObject
{
public:
    ImageObject();
    
    struct Distance {
        float distance; //meter
        int size;       //pixel
    };
    
    std::string name;
    int h_min;
    int h_max;
    int s_min;
    int s_max;
    int v_min;
    int v_max;
    Distance d1;
    Distance d2;
    
    float getDistance(int size);
};

#endif // IMAGEOBJECT_H
