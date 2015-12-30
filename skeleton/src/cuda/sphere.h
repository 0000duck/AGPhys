#pragma once

#include <glm/glm.hpp>

#include "cuda_typedefs.h"
#include "plane.h"

namespace CUDA{


class ALIGNMENT Sphere
{
public:
    vec3_t position;
    float radius;
    vec4_t color;
    vec3_t velocity;
    float mass;

    int sphereCollider;
    int planeCollider;

    int id;
    int nextInList;

    Sphere(){}

}ATTRIBUTES;

void resetSpheres(Sphere* spheres, int numberOfSpheres, int x, int z, const glm::vec3& corner, float distance);

void updateAllSpheresBruteForce(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);
void updateAllSpheresSortAndSweep(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt);
void updateAllSpheresLinkedCell(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt, const glm::vec3& dim_colDomain, const glm::vec3& offset_colDomain, float maxRadius);

}
