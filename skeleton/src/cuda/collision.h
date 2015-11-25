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
    Sphere* sphere;
    Plane* plane;

    bool isSphereIntersection; // true = sphere, false = plane

    bool intersects;

}__attribute__((packed));

#endif

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos1, float3 lastValidPos2, Sphere* sphere)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos1 = lastValidPos1;
    i.lastValidPos2 = lastValidPos2;
    i.sphere = sphere;
    i.isSphereIntersection = true;
    return i;
}

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos1, float3 lastValidPos2, Plane* plane)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos1 = lastValidPos1;
    i.lastValidPos2 = lastValidPos2;
    i.plane = plane;
    i.isSphereIntersection = false;
    return i;
}

__device__ inline IntersectionData make_intersectiondata(bool intersects)
{
    IntersectionData i;
    i.intersects = intersects;
    return i;
}

__device__ inline IntersectionData make_intersectiondata()
{
    return make_intersectiondata(false);
}

__device__ IntersectionData collideSpherePlane(Sphere* sphere, Plane* plane, float dt)
{
    Sphere updated = *sphere;
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

    return make_intersectiondata(intersects, colNormal, colTime, lastValidPos, plane->center, plane);
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

    return make_intersectiondata(true, colNormal, colTime, lastValidPos1, lastValidPos2, sphere2);
}

__device__ void resolveCollisionKinematically(Sphere* sphere, IntersectionData* intersection)
{
    sphere->impulse = reflect(sphere->impulse, intersection->colNormal);
    sphere->position = intersection->lastValidPos1 + 0.001 * intersection->colNormal;
}

__device__ void resolveCollisionSphereSphere(Sphere* sphere1, Sphere* sphere2, IntersectionData* intersection)
{
    float3 v1 = sphere1->impulse;
    float3 v2 = sphere2->impulse;
    float  m1 = sphere1->mass;
    float  m2 = sphere2->mass;

    float3 v1_normal = dot(v1, intersection->colNormal) / dot(intersection->colNormal, intersection->colNormal) * intersection->colNormal;
    float3 v1_tang   = v1 - v1_normal;

    float3 v2_normal = dot(v2, intersection->colNormal) / dot(intersection->colNormal, intersection->colNormal) * intersection->colNormal;
    float3 v2_tang   = v2 - v2_normal;

    float3 w1_normal, w2_normal;
    if (m1 == m2)
    {
        w1_normal = v2_normal;
        w2_normal = v1_normal;
    }
    else
    {
        w1_normal = (m1 - m2) / (m1 + m2) * v1_normal + (2 * m2) / (m1 + m2) * v2_normal;
        w2_normal = (2 * m1) / (m1 + m2) * v1_normal + (m2 - m1) / (m1 + m2) * v2_normal;
    }


    float lamda_shear = 0.5;
    float lamda_dashpot = 0;

    float3 newImpulse1 = v1_tang + w1_normal;
    float3 newImpulse2 = v2_tang + w2_normal;

    sphere1->position = intersection->lastValidPos1;
    sphere2->position = intersection->lastValidPos2;
    sphere1->impulse = newImpulse1;
    sphere2->impulse = newImpulse2;
}

__device__ void resolveCollisionSpherePlane(Sphere* sphere, Plane* plane, IntersectionData* intersection)
{
    float3 v_normal = dot(sphere->impulse, plane->normal) / dot(plane->normal, plane->normal) * plane->normal;
    float3 v_tang   = sphere->impulse - v_normal;

    float lamda_shear = 0.2;
    float lamda_dashpot = 0.1;

    float3 newImpulse = (1 - lamda_shear) * v_tang - (1 - lamda_dashpot) * v_normal;

    sphere->position = intersection->lastValidPos1;
    sphere->impulse = newImpulse;
}

__device__ inline void resolveCollisionDynamically(Sphere* sphere, IntersectionData* intersection)
{
    if (length(sphere->position - intersection->lastValidPos1) > 5) return;

    if (intersection->isSphereIntersection)
    {
        resolveCollisionSphereSphere(sphere, intersection->sphere, intersection);
    }
    else
    {
        resolveCollisionSpherePlane(sphere, intersection->plane, intersection);
    }
}

