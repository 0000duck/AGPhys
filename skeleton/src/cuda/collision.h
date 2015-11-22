#pragma once

#include "sphere.h"
#include "plane.h"
#include "helper_math.h"
#include "helper_cuda.h"

using namespace CUDA;

#ifdef __CUDACC__

class __align__(4) IntersectionData
{
public:
    float3 colNormal;
    float colTime;
    float3 lastValidPos1;
    float3 lastValidPos2;
    bool intersects;

}__attribute__((packed));

#endif

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos1, float3 lastValidPos2)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos1 = lastValidPos1;
    i.lastValidPos2 = lastValidPos2;
    return i;
}

__device__ inline IntersectionData make_intersectiondata()
{
    return make_intersectiondata(false, make_float3(0), 0.0f, make_float3(0), make_float3(0));
}

__device__ IntersectionData collideSpherePlane(Sphere* sphere, Plane* plane, float dt)
{
    Sphere updated = *sphere;
    updated.impulse += dt * make_float3(0, -1, 0); // gravity
    updated.position += dt * updated.impulse;

    bool intersects = dot(plane->normal, updated.position) <= plane->d + updated.radius * length(plane->normal);
    if (!intersects)
    {
        // return empty
        return make_intersectiondata();
    }

    float3 colNormal = plane->normal;
    float colTime = dt * ((dot(plane->normal, sphere->position) - plane->d - sphere->radius * length(plane->normal)) / (dot(plane->normal, sphere->position) - dot(plane->normal, updated.position)));
    float3 lastValidPos = sphere->position + sphere->impulse * colTime;

    return make_intersectiondata(intersects, colNormal, colTime, lastValidPos, plane->center);
}

__device__ IntersectionData collideSphereSphere(Sphere* sphere1, Sphere *sphere2)
{
    Sphere s1 = *sphere1;
    Sphere s2 = *sphere2;

    s1.impulse -= s2.impulse;
    s2.radius += s1.radius;
    s1.radius = 0;
    s2.impulse = make_float3(0);

    //float colTime = (dot())


    return make_intersectiondata();
}

