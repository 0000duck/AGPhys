#pragma once

#include <glm/glm.hpp>
#include "sphere.h"
#include "plane.h"

#ifdef __CUDACC__
#define ATTRIBUTES __attribute__((packed))
#define ALIGNMENT __align__(4)
#else
#define ATTRIBUTES
#define ALIGNMENT
#endif

namespace CUDA
{

struct ALIGNMENT Spring
{
    int index1;
    int index2;

    float hardness;
    float length;
} ATTRIBUTES;

class Cloth
{
public:
    void init(int m, int n, float springLength);
    void shutdown();
    void update(Sphere *spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, Sphere* colliders, int numberOfColliders, float dt, glm::vec4 fixated = glm::vec4(-1));

private:
    Spring* dev_springs;
    int numberOfSprings;
};



}
