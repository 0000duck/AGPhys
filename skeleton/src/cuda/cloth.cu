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

__device__ bool isFix(Sphere& s, int4 fix)
{
    return s.id == fix.x || s.id == fix.y || s.id == fix.z || s.id == fix.w;
}

/// ----------------------------------------------------------------------------------------------------------------
/// ------------------------------------------------- SPRING BASED -------------------------------------------------
/// ----------------------------------------------------------------------------------------------------------------

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
                                    float dt, int4 fix)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

        if (!isFix(s, fix))
        {
            s.position += s.velocity * dt;
            s.velocity.y -= 9.81f * dt;

            s.velocity *= 0.999;


            collideWithPlanes(s, planes, numberOfPlanes, dt);
            collideWithSpheres(s, colliders, numberOfColliders);
        }
        else
        {
            s.color = make_float4(0.f, 0.f, 1.f, 1.f);
        }

    }
}

/// ------------------------------------------------------------------------------------------------------------------
/// ------------------------------------------------- POSITION BASED -------------------------------------------------
/// ------------------------------------------------------------------------------------------------------------------

__global__ void estimatePositions(Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, Sphere* colliders, int numberOfColliders,
                                  float dt, int4 fix)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

        if (!isFix(s, fix))
        {
            collideWithPlanes(s, planes, numberOfPlanes, dt);
            collideWithSpheres(s, colliders, numberOfColliders);

            s.velocity.y -= 9.81f * dt;
            s.velocity *= 0.999;
            s.estimatedPosition = s.position + s.velocity * dt;
        }
        else
        {
            s.estimatedPosition = s.position;
            s.color = make_float4(0.f, 0.f, 1.f, 1.f);
        }
    }
}

__device__ void updateEstimatedPosition(Sphere& s, float3 c)
{
    atomicAdd(&s.estimatedPosition.x, c.x);
    atomicAdd(&s.estimatedPosition.y, c.y);
    atomicAdd(&s.estimatedPosition.z, c.z);
}

__global__ void projectConstraints(LinearConstraint* constraints, int numberOfConstraints, BendingConstraint* b_constraints, int numberOfBendingConstraints, Sphere* spheres, int4 fix)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfConstraints)
    {

        LinearConstraint& constraint  = constraints[tid];
        Sphere& s1      = spheres[constraint.index1];
        Sphere& s2      = spheres[constraint.index2];

        float w1 = 1.f / s1.mass;
        float w2 = 1.f / s2.mass;
        float3& p1 = s1.estimatedPosition;
        float3& p2 = s2.estimatedPosition;
        float distance = length(p1 - p2);
        float3 dir = (p1 - p2) / distance;
        bool isFix1 = isFix(s1, fix);
        bool isFix2 = isFix(s2, fix);

        float3 correction1 = -w1 / (w1 + w2) * (distance - constraint.d) * dir;
        float3 correction2 = w2 / (w1 + w2) * (distance - constraint.d) * dir;

        float damp = .2f;

        if (isFix1)
        {
            updateEstimatedPosition(s2, 2.f * correction2 * damp);
        }
        else if (isFix2)
        {
            updateEstimatedPosition(s1, 2.f * correction1 * damp);
        }
        else
        {
            updateEstimatedPosition(s1, correction1 * damp);
            updateEstimatedPosition(s2, correction2 * damp);
        }
    }
    else if (tid < numberOfConstraints + numberOfBendingConstraints)
    {
        BendingConstraint& constraint = b_constraints[tid - numberOfConstraints];
        Sphere& s1 = spheres[constraint.index1];
        Sphere& s2 = spheres[constraint.index2];
        Sphere& s3 = spheres[constraint.index3];
        Sphere& s4 = spheres[constraint.index4];

        /*
         *   s1 --- s2
         *    |   /  |
         *    |  /   |
         *   s3 --- s4
         */

        float3 p1 = make_float3(0.f);
        float3& p2 = s2.estimatedPosition;
        float3& p3 = s3.estimatedPosition;
        float3& p4 = s4.estimatedPosition;

        float w1 = 1.f / s1.mass;
        float w2 = 1.f / s2.mass;
        float w3 = 1.f / s3.mass;
        float w4 = 1.f / s4.mass;

        float3 n1 = normalize(cross(p2, p3));
        float3 n2 = normalize(cross(p2, p4));

        float3 q3 = (cross(p2, n2) + cross(n1, p2) * constraint.d) / length(cross(p2, p3));
        float3 q4 = (cross(p2, n1) + cross(n2, p2) * constraint.d) / length(cross(p2, p4));
        float3 q2 = -1.f * (cross(p3, n2) + cross(n1, p3) * constraint.d) / length(cross(p2, p3)) - (cross(p4, n1) + cross(n2, p4) * constraint.d) / length(cross(p2, p4));
        float3 q1 = -q2 - q3 - q4;

        float phi = 1.f;

        float sum = w1 * dot(q1, q1) + w2 * dot(q2, q2) + w3 * dot(q3, q3) + w4 * dot(q4, q4);

        float3 correction1 = -(w1 * sqrt(1.0 - constraint.d) * (acos(constraint.d) - phi)) / sum * q1;
        float3 correction2 = -(w2 * sqrt(1.0 - constraint.d) * (acos(constraint.d) - phi)) / sum * q2;
        float3 correction3 = -(w3 * sqrt(1.0 - constraint.d) * (acos(constraint.d) - phi)) / sum * q3;
        float3 correction4 = -(w4 * sqrt(1.0 - constraint.d) * (acos(constraint.d) - phi)) / sum * q4;

        updateEstimatedPosition(s1, correction1 / 100);
        updateEstimatedPosition(s2, correction2 / 100);
        updateEstimatedPosition(s3, correction3 / 100);
        updateEstimatedPosition(s4, correction4 / 100);

    }
}

__global__ void setPositions(Sphere* spheres, int numberOfSpheres, float dt, int4 fix)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

        if (!isFix(s, fix))
        {
            s.velocity = (s.estimatedPosition - s.position) / dt;
            s.position = s.estimatedPosition;
        }
    }
}

void Cloth::init(int m, int n, float springLength)
{
    int sm = m - 1;
    int sn = n - 1;
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


    assert(springs.size() == m*sn + n*sm);

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
            s.hardness = hardness;
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
            s.hardness = hardness;
            s.index1 = y * n + x;
            s.index2 = (y + 2) * n + x;
            springs.push_back(s);
        }
    }

    numberOfSprings = springs.size();


    // CONSTRAINTS
    std::vector<LinearConstraint> constraints;
    for (int y = 0; y < m; ++y)
    {
        for (int x = 0; x < sn; ++x)
        {
            LinearConstraint c;
            c.d = springLength;
            c.index1 = y * n + x;
            c.index2 = y * n + x + 1;
            constraints.push_back(c);
        }
    }
    for (int y = 0; y < sm; ++y)
    {
        for (int x = 0; x < n; ++x)
        {
            LinearConstraint c;
            c.d = springLength;
            c.index1 = y * n + x;
            c.index2 = (y + 1) * n + x;
            constraints.push_back(c);
        }
    }

    numberOfConstraints = constraints.size();

    std::vector<BendingConstraint> b_constraints;
    for (int y = 0; y < sm; ++y)
    {
        for (int x = 0; x < sn; ++x)
        {
            BendingConstraint c;
            c.d = 1.f;
            c.index1 = y * n + x;
            c.index2 = y * n + x + 1;
            c.index3 = (y+1) * n + x;
            c.index4 = (y+1) * n + x + 1;
            b_constraints.push_back(c);
        }
    }

    numberOfBendingConstraints = b_constraints.size();


    cudaMalloc(&dev_springs, sizeof(Spring) * springs.size());
    cudaMemcpy(dev_springs, &springs[0], sizeof(Spring) * springs.size(), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_constraints, sizeof(LinearConstraint) * constraints.size());
    cudaMemcpy(dev_constraints, &constraints[0], sizeof(LinearConstraint) * constraints.size(), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_bconstraints, sizeof(BendingConstraint) * b_constraints.size());
    cudaMemcpy(dev_bconstraints, &b_constraints[0], sizeof(BendingConstraint) * b_constraints.size(), cudaMemcpyHostToDevice);

}

void Cloth::shutdown()
{
    cudaFree(dev_bconstraints);
    cudaFree(dev_constraints);
    cudaFree(dev_springs);
}

void Cloth::updateWithSprings(Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, Sphere* colliders, int numberOfColliders, float dt, glm::vec4 fixated)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    // INTEGRATE
    int4 fix = make_int4(fixated.x, fixated.y, fixated.z, fixated.w);
    integrateMassPoints<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, planes, numberOfPlanes, colliders, numberOfColliders, dt, fix);

    blocks = numberOfSprings / threadsPerBlock + 1;

    computeForces<<<blocks, threadsPerBlock>>>(dev_springs, numberOfSprings, spheres);
}

void Cloth::updatePositionBased(Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, Sphere* colliders, int numberOfColliders, float dt, glm::vec4 fixated)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    // INTEGRATE, ESTIMATE POSITIONS & DAMP VELOCITY
    int4 fix = make_int4(fixated.x, fixated.y, fixated.z, fixated.w);
    estimatePositions<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, planes, numberOfPlanes, colliders, numberOfColliders, dt, fix);

    blocks = (numberOfConstraints + numberOfBendingConstraints) / threadsPerBlock + 1;

    for (int i = 0; i < 10; ++i)
    {
        projectConstraints<<<blocks, threadsPerBlock>>>(dev_constraints, numberOfConstraints, dev_bconstraints, numberOfBendingConstraints, spheres, fix);
    }

    blocks = numberOfSpheres / threadsPerBlock + 1;

    setPositions<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt, fix);
}

}
