#include "imageobject.h"

ImageObject::ImageObject()
{
}

float ImageObject::getDistance(int size) {
    float a = (d1.size - d2.size) / (d1.distance - d2.distance),
          b = d2.size - (d2.distance * a);
    
    return (size - b) / a;
}
