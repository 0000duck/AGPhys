#pragma once

#include "cuda_typedefs.h"
#include "plane.h"

namespace CUDA{


class ALIGNMENT Sphere
{
public:
    vec3_t position;
    float radius;
    vec4_t color;
    vec3_t impulse;

    Sphere(){}

}ATTRIBUTES;


void updateAllSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);

}
