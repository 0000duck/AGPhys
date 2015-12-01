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
    float3 lastValidPos;
    Sphere* sphere;
    Plane* plane;

    bool isSphereIntersection; // true = sphere, false = plane

    bool intersects;

}__attribute__((packed));

#endif

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos, Sphere* sphere)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos = lastValidPos;
    i.sphere = sphere;
    i.isSphereIntersection = true;
    return i;
}

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos, Plane* plane)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos = lastValidPos;
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



/// ---------------------- COLLISION DETECTION ----------------------

__device__ IntersectionData collideSpherePlane(Sphere* sphere, Plane* plane, float dt)
{
    bool intersects = dot(plane->normal, sphere->newPos) <= plane->d + sphere->radius * length(plane->normal);
    if (!intersects)
    {
        // return empty
        return make_intersectiondata();
    }

    float3 colNormal = normalize(plane->normal);
    float colTime = dt * ((dot(plane->normal, sphere->position) - plane->d - sphere->radius * length(plane->normal)) / (dot(plane->normal, sphere->position) - dot(plane->normal, sphere->newPos)));
    float3 lastValidPos = sphere->position + sphere->impulse * colTime;

    return make_intersectiondata(intersects, colNormal, colTime, lastValidPos, plane);
}

__device__ float computeCollisionTime(Sphere* sphere1, Sphere* sphere2, float dt)
{
    // assumption: spheres collide after update
    Sphere s1 = *sphere1;
    Sphere s2 = *sphere2;

    s1.impulse -= s2.impulse;
    s2.radius += s1.radius;
    s1.radius = 0;
    s2.impulse = make_float3(0);

    float colTime = (dot(s1.impulse, (s2.position - s1.position))
                     - sqrt(pow(dot(s1.impulse, (s1.position - s2.position)), 2) - dot(s1.impulse, s1.impulse) * (dot(s1.position - s2.position, s1.position - s2.position) - s2.radius)))
                     / dot(s1.impulse, s1.impulse);

    return colTime;
}

__device__ IntersectionData collideSphereSphere(Sphere* sphere1, Sphere *sphere2, float dt)
{
    bool colBeforeUpdate = length(sphere1->position - sphere2->position) <= sphere1->radius + sphere2->radius;
    bool colAfterUpdate = length(sphere1->newPos - sphere2->newPos) <= sphere1->radius + sphere2->radius;

    // NO INTERSECTION
    if (!colBeforeUpdate && !colAfterUpdate)
    {
        return make_intersectiondata();
    }

    // ONLY BEFORE UPDATE
    if (colBeforeUpdate && !colAfterUpdate)
    {
        return make_intersectiondata();
    }

    // ONLY AFTER UPDATE
    if (!colBeforeUpdate && colAfterUpdate)
    {
        float colTime = computeCollisionTime(sphere1, sphere2, dt);
        float3 lastValidPos1 = sphere1->position + sphere1->impulse * colTime * 0.99f;
        float3 lastValidPos2 = sphere2->position + sphere2->impulse * colTime * 0.99f;
        float3 colNormal = normalize(lastValidPos1 - lastValidPos2);

        return make_intersectiondata(true, colNormal, colTime, lastValidPos1, sphere2);
    }

    // BOTH BEFORE AND AFTER UPDATE

    float distanceBeforeUpdate = length(sphere1->position - sphere2->position);
    float distanceAfterUpdate = length(sphere1->newPos - sphere2->newPos);

    // depart
    if (distanceAfterUpdate > distanceBeforeUpdate)
    {
        return make_intersectiondata();
    }
    // approach
    else if (distanceAfterUpdate < distanceBeforeUpdate)
    {
        float3 colNormal = normalize(sphere1->position - sphere2->position);
        return make_intersectiondata(true, colNormal, 0, sphere1->position, sphere2);
    }
    // same impulse
    else
    {
        return make_intersectiondata();
    }

}



/// ---------------------- COLLISION RESOLUTION ----------------------

__device__ void resolveCollisionKinematically(Sphere* sphere, IntersectionData* intersection)
{
    sphere->impulse  = 0.9 * reflect(sphere->impulse, intersection->colNormal);
    sphere->newPos      = intersection->lastValidPos;
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

    float3 newImpulse1 = v1_tang + w1_normal;
    float3 newImpulse2 = v2_tang + w2_normal;

    sphere1->newPos = intersection->lastValidPos;
    sphere1->impulse = newImpulse1;
}

__device__ void resolveCollisionSpherePlane(Sphere* sphere, Plane* plane, IntersectionData* intersection)
{
    float3 v_normal = dot(sphere->impulse, plane->normal) / dot(plane->normal, plane->normal) * plane->normal;
    float3 v_tang   = sphere->impulse - v_normal;

    float lamda_shear = 0.2;
    float lamda_dashpot = 0.1;

    float3 newImpulse = (1 - lamda_shear) * v_tang - (1 - lamda_dashpot) * v_normal;


    sphere->newPos  = intersection->lastValidPos;
    sphere->impulse = newImpulse;
}

__device__ inline void resolveCollisionDynamically(Sphere* sphere, IntersectionData* intersection)
{
    if (intersection->isSphereIntersection)
    {
        resolveCollisionSphereSphere(sphere, intersection->sphere, intersection);
    }
    else
    {
        resolveCollisionSpherePlane(sphere, intersection->plane, intersection);
    }
}
