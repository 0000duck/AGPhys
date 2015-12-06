#define KINEMATIC
#define GRAVITY

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


#include "sphere.h"
#include "collision.h"
#include "timing.h"

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

__global__ void integrateSpheres(Sphere* spheres, int numberOfSpheres, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

#ifdef GRAVITY
        s.impulse  += dt * make_float3(0, -1, 0); // gravity, breaks everything......
#endif
        s.position += dt * s.impulse;

        // DEBUG
        //s.color = make_float4(s.impulse) / 5 + make_float4(1, 1, 1, 0);
    }

}

__global__ void collidePlanes(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        for (int p = 0; p < numberOfPlanes; ++p)
        {
            Plane& plane = planes[p];
            float penetration = collideSpherePlane(sphere, plane);
            if (penetration != -1.0f)
            {
                //sphere.color = make_float4(1);

#ifdef KINEMATIC
                kinematicCollisionResponseSpherePlane(sphere, plane, penetration);
#else
                dynamicCollisionResponseSpherePlane(sphere, plane, penetration);
#endif
            }
        }
    }
}

/// --------------------------------------------------------------------- BRUTE FORCE ---------------------------------------------------------------------

__global__ void collideSpheresBruteForce(Sphere* spheres, int numberOfSpheres)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        Sphere* collider = NULL;
        float max = -1.0f;
        for (int s = 0; s < tid; ++s)
        {
            if (s == tid) continue;
            Sphere& other = spheres[s];
            float penetration = collideSphereSphere(sphere, other);
            if (penetration != -1.0f)
            {
                if (penetration > max)
                {
                    collider = &other;
                    max = penetration;
                }
            }
        }

        if (max > -1.0f)
        {

#ifdef KINEMATIC
                kinematicCollisionResponseSphereSphere(sphere, *collider, max);
#else
                elasticCollision(sphere, *collider, max);
#endif

        }

    }
}


/// --------------------------------------------------------------------- SORT & SWEEP --------------------------------------------------------------------

struct ALIGNMENT AxisProjection
{
    float   value;
    bool    startPoint;
    int     sphereID;

} ATTRIBUTES;

struct FillWithStartPoints
{
    __host__ __device__ AxisProjection operator()(const Sphere& s)
    {
        AxisProjection b;
        b.startPoint = true;
        b.value = s.position.x - s.radius;
        b.sphereID = s.id;
        return b;
    }
};

struct FillWithEndPoints
{
    __host__ __device__ AxisProjection operator()(const Sphere& s)
    {
        AxisProjection e;
        e.startPoint = false;
        e.value = s.position.x + s.radius;
        e.sphereID = s.id;
        return e;
    }
};

struct Sort
{
    __host__ __device__ bool operator()(const AxisProjection& a1, const AxisProjection& a2)
    {
        return a1.value < a2.value;
    }
};



__global__ void collideSpheresSortAndSweep(AxisProjection* projections, Sphere* spheres, int numberOfProjections)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfProjections)
    {
        AxisProjection& projection = projections[tid];
        if (!projection.startPoint)
        {
            return;
        }

        Sphere* collider = NULL;
        float max = -1.0f;

        for (int i = tid + 1;; ++i)
        {
            AxisProjection& other = projections[i];

            if (other.sphereID == projection.sphereID)
            {
                // endpoint of own sphere
                break;
            }

            if (!other.startPoint || other.sphereID > projection.sphereID)
            {
                continue;
            }

            float penetration = collideSphereSphere(spheres[projection.sphereID], spheres[other.sphereID]);
            if (penetration != -1.0f)
            {
                if (penetration > max)
                {
                    collider = &spheres[other.sphereID];
                    max = penetration;
                }
            }
        }

        if (max > -1.0f)
        {

#ifdef KINEMATIC
            kinematicCollisionResponseSphereSphere(spheres[projection.sphereID], *collider, max);
#else
            elasticCollision(spheres[projection.sphereID], *collider, max);
#endif

        }
    }
}


/// --------------------------------------------------------------------- LINKED CELL ---------------------------------------------------------------------

__global__ void collideSpheresLinkedCell(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes)
{

}








/// ------------------------------------------------------------------------- CPU -------------------------------------------------------------------------

void resetSpheres(Sphere* spheres, int numberOfSpheres, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    resetSpheresGrid<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, x, z, cornerX, cornerY, cornerZ, distance);
}


static float accTime = 0.0f;
static float accDts = 0.0f;
static int numberOfSamples = 0;


void updateAllSpheresBruteForce(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);


    startTiming();
    collidePlanes<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);
    collideSpheresBruteForce<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres);
    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;


    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "BRUTE FORCE: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;
        std::cout.flush();

        accTime = 0.0f;
        accDts  = 0.0f;
        numberOfSamples = 0;
    }
}

void updateAllSpheresSortAndSweep(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);


    startTiming();

    collidePlanes<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);

    thrust::device_vector<AxisProjection> projectionVector(numberOfSpheres * 2);

    thrust::device_ptr<Sphere> dev_ptr1(spheres);
    thrust::device_ptr<Sphere> dev_ptr2(spheres + numberOfSpheres);
    thrust::transform(dev_ptr1, dev_ptr2, projectionVector.begin(), FillWithStartPoints());
    thrust::transform(dev_ptr1, dev_ptr2, projectionVector.begin() + numberOfSpheres, FillWithEndPoints());
    thrust::sort(projectionVector.begin(), projectionVector.end(), Sort());

    AxisProjection* projectionPtr = thrust::raw_pointer_cast(projectionVector.data());

    collideSpheresSortAndSweep<<<blocks * 2, threadsPerBlock>>>(projectionPtr, spheres, numberOfSpheres * 2);

    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;

    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "SORT & SWEEP: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;

        accTime = 0.0f;
        accDts = 0.0f;
        numberOfSamples = 0;
    }
}

void updateAllSpheresLinkedCell(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);


    startTiming();
    collideSpheresLinkedCell<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);
    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;

    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "LINKED CELL: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;

        accTime = 0.0f;
        accDts = 0.0f;
        numberOfSamples = 0;
    }
}







}
