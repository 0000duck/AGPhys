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
    float mass;
    int id;

    Sphere(){}

}ATTRIBUTES;

void resetSpheres(Sphere* spheres, int numberOfSpheres, int x, int z, float cornerX, float cornerY, float cornerZ, float distance);

void updateAllSpheresBruteForce(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);
void updateAllSpheresSortAndSweep(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);
void updateAllSpheresLinkedCell(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);

}
