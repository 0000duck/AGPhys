#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

#include "sphere.h"
#include "collision.h"

namespace CUDA {


__global__ void updateSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;

    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

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
                }
            }

        }


        // UPDATE SPHERE
        if (firstIntersection.intersects)
        {
            sphere.impulse += firstIntersection.colTime * make_float3(0, -1, 0);
            sphere.impulse = 0.75 * reflect(sphere.impulse, firstIntersection.colNormal);
            sphere.position = firstIntersection.lastValidPos1;
        }
        else
        {
            sphere.impulse += dt * make_float3(0, -1, 0);
            sphere.position += dt * sphere.impulse;
        }

    }
}


void updateAllSpheres(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    updateSpheres<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes, dt);
}


}

