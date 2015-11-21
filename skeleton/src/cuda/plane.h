#pragma once

#include "cuda_typedefs.h"

namespace CUDA{


class ALIGNMENT Plane
{
public:
    vec3_t center;
    vec3_t normal;
    vec3_t d1;
    vec3_t d2;
    float d;

    Plane(){}

}ATTRIBUTES;

}
