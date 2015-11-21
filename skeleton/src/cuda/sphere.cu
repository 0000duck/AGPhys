#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

#include "sphere.h"

namespace CUDA {

__device__ void collideSpherePlane(Sphere* sphere, Plane* plane)
{

}

__global__ void updateSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;

    if (tid < numberOfSpheres)
    {
        spheres[tid].position += dt * spheres[tid].impulse;
    }
}


void updateAllSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    updateSpheres<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes, dt);
}


}

