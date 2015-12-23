#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

// thrust
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>


#include "rigidbody.h"

namespace CUDA
{

RigidBody* dev_ptr;
int numberOfBodies;

void initRigidBodies(RigidBody* host_bodies, int size)
{
    cudaMalloc(&dev_ptr, sizeof(RigidBody) * size);
    cudaMemcpy(dev_ptr, host_bodies, sizeof(RigidBody) * size, cudaMemcpyHostToDevice);
    numberOfBodies = size;
}

void shutdownRigidBodies()
{
    cudaFree(dev_ptr);
}

__global__ void getPosAndRot(RigidBody* bodies, vec3_t* pos_ptr, quat_t* rot_ptr, int numberOfBodies)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfBodies)
    {
        RigidBody& rb = bodies[tid];
        pos_ptr[tid] = rb.position;
        rot_ptr[tid] = rb.rotation;
    }
}

void getOrientationData(std::vector<glm::vec3>& pos, std::vector<glm::quat>& rot)
{
    int threadsPerBlock = 128;
    int blocks = numberOfBodies / threadsPerBlock + 1;

    vec3_t* pos_ptr;
    quat_t* rot_ptr;

    cudaMalloc(&pos_ptr, sizeof(vec3_t) * numberOfBodies);
    cudaMalloc(&rot_ptr, sizeof(quat_t) * numberOfBodies);

    getPosAndRot<<<blocks, threadsPerBlock>>>(dev_ptr, pos_ptr, rot_ptr, numberOfBodies);

    cudaMemcpy(&pos[0], pos_ptr, sizeof(vec3_t) * numberOfBodies, cudaMemcpyDeviceToHost);
    cudaMemcpy(&rot[0], rot_ptr, sizeof(quat_t) * numberOfBodies, cudaMemcpyDeviceToHost);

    cudaFree(pos_ptr);
    cudaFree(rot_ptr);
}

__global__ void updateBodies(RigidBody* bodies, int numberOfBodies, Sphere* spheres, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfBodies)
    {
        RigidBody& rb = bodies[tid];

        uint sphereStart = 0;
        uint sphereEnd = 0;
        for (int i = 0; i < tid; ++i)
        {
            sphereStart += bodies[i].numberOfSpheres;
        }
        sphereEnd = sphereStart + rb.numberOfSpheres;

        rb.linearVelocity += dt * make_float3(0.0, -0.1f, 0.0f);
        rb.position += dt * rb.linearVelocity;
    }
}

void updateRigidBodies(Sphere* spheres, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfBodies / threadsPerBlock + 1;

    updateBodies<<<blocks, threadsPerBlock>>>(dev_ptr, numberOfBodies, spheres, dt);
}

}
