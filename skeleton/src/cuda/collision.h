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

    float3 colNormal = normalize(plane->normal);
    float colTime = dt * ((dot(plane->normal, sphere->position) - plane->d - sphere->radius * length(plane->normal)) / (dot(plane->normal, sphere->position) - dot(plane->normal, updated.position)));
    float3 lastValidPos = sphere->position + sphere->impulse * colTime;

    return make_intersectiondata(intersects, colNormal, colTime, lastValidPos, plane->center);
}

__device__ IntersectionData collideSphereSphere(Sphere* sphere1, Sphere *sphere2, float dt)
{
    Sphere s1 = *sphere1;
    Sphere s2 = *sphere2;

    if (s1.impulse.x == s2.impulse.x && s1.impulse.y == s2.impulse.y && s1.impulse.z == s2.impulse.z)
    {
        return make_intersectiondata();
    }

    s1.impulse -= s2.impulse;
    s2.radius += s1.radius;
    s1.radius = 0;
    s2.impulse = make_float3(0);

    bool no_int1 = dot((s2.position - s1.position), s1.impulse) < 0;
    bool no_int2 = dot((s2.position - s1.position), (s2.position - s1.position)) - (pow(dot((s2.position - s1.position), s1.impulse), 2)) / (dot(s1.impulse, s1.impulse)) > s2.radius * s2.radius;

    bool intersects = !no_int1 && !no_int2;
    if (!intersects)
    {
        return make_intersectiondata();
    }

    // ugliest. equation. ever.
    float colTime = (dot(s1.impulse, (s2.position - s1.position))
                     - sqrt(pow(dot(s1.impulse, (s1.position - s2.position)), 2) - dot(s1.impulse, s1.impulse) * (dot(s1.position - s2.position, s1.position - s2.position) - s2.radius)))
                     / dot(s1.impulse, s1.impulse);

    if (colTime > dt)
    {
        return make_intersectiondata(); // no intersection in this frame
    }

    float3 lastValidPos1 = sphere1->position + sphere1->impulse * colTime;
    float3 lastValidPos2 = sphere2->position + sphere2->impulse * colTime;
    float3 colNormal = normalize(lastValidPos1 - lastValidPos2);

    return make_intersectiondata(true, colNormal, colTime, lastValidPos1, lastValidPos2);
}

