#pragma once

#include "cuda_typedefs.h"
#include "sphere.h"

#include <glm/glm.hpp>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <vector>

namespace CUDA{


class ALIGNMENT RigidBody
{
public:
    vec3_t position;
    quat_t rotation;
    float  mass;
    vec3_t linearVelocity;
    vec3_t angularVelocity;

    int    numberOfSpheres;

    RigidBody(){}


}ATTRIBUTES;

void initRigidBodies(RigidBody* host_bodies, int size);
void shutdownRigidBodies();
void getOrientationData(std::vector<glm::vec3>& pos, std::vector<glm::quat>& rot);


void updateRigidBodies(Sphere* spheres, float dt);

}
