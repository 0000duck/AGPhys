#include "saiga/geometry/triangle.h"


vec3 Triangle::center(){
    return (a+b+c)/vec3(3);
}

