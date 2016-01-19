#include <vector>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"
#include <iostream>


#include "cloth.h"
#include "collision.h"


namespace CUDA
{

__device__ inline void atomicAddVelocity(Sphere& s, float3 v)
{
    atomicAdd(&s.velocity.x, v.x);
    atomicAdd(&s.velocity.y, v.y);
    atomicAdd(&s.velocity.z, v.z);
}

__global__ void resetForces(Sphere* spheres, int numberOfSpheres)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];
        s.force = make_float3(0.f);
    }
}

__global__ void computeForces(Spring* springs, int numberOfSprings, Sphere* spheres)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSprings)
    {
        Spring& spring  = springs[tid];
        Sphere& s1      = spheres[spring.index1];
        Sphere& s2      = spheres[spring.index2];

        float l = length(s2.position - s1.position);
        float3 deformation = (s2.position - s1.position) / l * (l - spring.length);
        float3 force = -spring.hardness * deformation;

        atomicAddVelocity(s1, -force / s1.mass);
        atomicAddVelocity(s2, force / s1.mass);
    }
}

__device__ void collideWithPlanes(Sphere& sphere, Plane* planes, int numberOfPlanes, float dt)
{
    for (int p = 0; p < numberOfPlanes; ++p)
    {
        Plane& plane = planes[p];
        float penetration = collideSpherePlane(sphere, plane);
        if (penetration != -1.0f)
        {
            //sphere.color = make_float4(1);

            dynamicCollisionResponseSpherePlane(sphere, plane, penetration, dt);
        }
    }
}

__device__ void collideWithSpheres(Sphere& sphere, Sphere* colliders, int numberOfColliders)
{
    float max = -1.0f;
    Sphere* maxCollider = NULL;
    for (int s = 0; s < numberOfColliders; ++s)
    {
        Sphere& collider = colliders[s];
        float penetration = collideSphereSphere(sphere, collider);

        if (penetration > max)
        {
            maxCollider = &collider;
            max = penetration;
        }
    }

    if (max > -1.0f)
    {
        elasticCollision(sphere, *maxCollider, max);
        sphere.velocity *= 0.5f;
        maxCollider->velocity *= 0.5f;
    }
}

__global__ void integrateMassPoints(Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, Sphere* colliders, int numberOfColliders,
                                    float dt, int fixated0, int fixated1, int fixated2, int fixated3)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

        if (s.id != fixated0 && s.id != fixated1 && s.id != fixated2 && s.id != fixated3)
        {
            s.position += s.velocity * dt;
            s.velocity.y -= 9.81f * dt;

            s.velocity = s.velocity * 0.999;
        }
        else
        {
            s.color = make_float4(0.f, 0.f, 1.f, 1.f);
        }

        collideWithPlanes(s, planes, numberOfPlanes, dt);
        collideWithSpheres(s, colliders, numberOfColliders);
    }
}

void Cloth::init(int m, int n, float springLength)
{
    int sm = m - 1;
    int sn = n - 1;
    this->numberOfSprings = m*sn + n*sm;
    float hardness = 10.f;

    std::vector<Spring> springs;

    // NEXT NEIGHBORS
    for (int y = 0; y < m; ++y)
    {
        for (int x = 0; x < sn; ++x)
        {
            Spring s;
            s.length = springLength;
            s.hardness = hardness;
            s.index1 = y * n + x;
            s.index2 = y * n + x + 1;
            springs.push_back(s);
        }
    }
    for (int y = 0; y < sm; ++y)
    {
        for (int x = 0; x < n; ++x)
        {
            Spring s;
            s.length = springLength;
            s.hardness = hardness;
            s.index1 = y * n + x;
            s.index2 = (y + 1) * n + x;
            springs.push_back(s);
        }
    }


    assert(springs.size() == numberOfSprings);

    // DIAGONAL
    float diagonalLength = (float)sqrt(springLength*springLength*2);
    for (int y = 0; y < sm; ++y)
    {
        for (int x = 0; x < sn; ++x)
        {
            Spring s;
            s.length = diagonalLength;
            s.hardness = hardness;
            s.index1 = y * n + x;
            s.index2 = (y + 1) * n + x + 1;
            springs.push_back(s);
        }
    }
    for (int y = 0; y < sm; ++y)
    {
        for (int x = 1; x < n; ++x)
        {
            Spring s;
            s.length = diagonalLength;
            s.hardness = hardness;
            s.index1 = y * n + x;
            s.index2 = (y + 1) * n + x - 1;
            springs.push_back(s);
        }
    }

    // DOUBLE NEIGHBOR
    for (int y = 0; y < m; ++y)
    {
        for (int x = 0; x < n-2; ++x)
        {
            Spring s;
            s.length = springLength * 2;
            s.hardness = hardness * 3;
            s.index1 = y * n + x;
            s.index2 = y * n + x + 2;
            springs.push_back(s);
        }
    }
    for (int y = 0; y < m-2; ++y)
    {
        for (int x = 0; x < n; ++x)
        {
            Spring s;
            s.length = springLength * 2;
            s.hardness = hardness * 3;
            s.index1 = y * n + x;
            s.index2 = (y + 2) * n + x;
            springs.push_back(s);
        }
    }


    cudaMalloc(&dev_springs, sizeof(Spring) * springs.size());
    cudaMemcpy(dev_springs, &springs[0], sizeof(Spring) * springs.size(), cudaMemcpyHostToDevice);

}

void Cloth::shutdown()
{
    cudaFree(dev_springs);
}

void Cloth::update(Sphere* spheres, int numberOfSpheres, Plane *planes, int numberOfPlanes, Sphere *colliders, int numberOfColliders, float dt, glm::vec4 fixated)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    // INTEGRATE
    integrateMassPoints<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, planes, numberOfPlanes, colliders, numberOfColliders, dt, fixated.x, fixated.y, fixated.z, fixated.w);


    //resetForces<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres);

    blocks = numberOfSprings / threadsPerBlock + 1;

    computeForces<<<blocks, threadsPerBlock>>>(dev_springs, numberOfSprings, spheres);
}

}
