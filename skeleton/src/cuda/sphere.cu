#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

#include "sphere.h"
#include "collision.h"

namespace CUDA {

__global__ void resetSpheresGrid(Sphere* spheres, int numberOfSpheres, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        int layerSize = x * z;
        int yPos = tid / layerSize;
        int normId = tid - yPos * layerSize;

        int xPos = normId % x;
        int zPos = (normId - xPos) / x;

        spheres[tid].position.x = xPos * distance + cornerX;
        spheres[tid].position.y = yPos * distance + cornerY;
        spheres[tid].position.z = zPos * distance + cornerZ;
    }
}


__global__ void updateSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;

    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];


        sphere.impulse += dt * make_float3(0.0f, -1.0f, 0.0f); // gravity

        IntersectionData firstIntersection = make_intersectiondata();

        // COLLIDE PLANES
        for (int p = 0; p < numberOfPlanes; ++p)
        {
            Plane& plane = planes[p];

            IntersectionData currentIntersection = collideSpherePlane(&sphere, &plane, dt);

            if (currentIntersection.intersects)
            {
                if (!firstIntersection.intersects || currentIntersection.colTime < firstIntersection.colTime)
                {
                    firstIntersection = currentIntersection;
                }
            }
        }

        int sphereIndex = 0;
        // TODO: COLLIDE SPHERES
        for (int s = 0; s < numberOfSpheres; ++s)
        {
            if (s == tid) continue; // same sphere

            Sphere& other = spheres[s];
            IntersectionData currentIntersection = collideSphereSphere(&sphere, &other, dt);

            if (currentIntersection.intersects)
            {
                if (!firstIntersection.intersects || currentIntersection.colTime < firstIntersection.colTime)
                {
                    firstIntersection = currentIntersection;
                    sphereIndex = s;
                }
            }

        }


        // UPDATE SPHERE
        if (firstIntersection.intersects)
        {
            if (firstIntersection.isSphereIntersection)
            {
                // only update if lower index
                if (sphereIndex > tid)
                {
                    //resolveCollisionKinematically(&sphere, &firstIntersection);
                    resolveCollisionDynamically(&sphere, &firstIntersection);
                }

            }
            else {
                //sphereresolveCollisionKinematically(&sphere, &firstIntersection);
                resolveCollisionDynamically(&sphere, &firstIntersection);
            }
        }
        else
        {
            // just move
            sphere.position += dt * sphere.impulse;
        }


    }
}

void resetSpheres(Sphere* spheres, int numberOfSpheres, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    resetSpheresGrid<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, x, z, cornerX, cornerY, cornerZ, distance);
}

void updateAllSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    updateSpheres<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes, dt);
}


}

