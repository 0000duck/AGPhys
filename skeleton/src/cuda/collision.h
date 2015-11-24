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
    float penetration;
    float3 relSpeed;
    bool intersects;

}__attribute__((packed));

#endif

__device__ inline IntersectionData make_intersectiondata(bool intersects, float3 colNormal, float colTime, float3 lastValidPos1, float3 lastValidPos2, float penetration, float3 relSpeed)
{
    IntersectionData i;
    i.intersects = intersects;
    i.colNormal = colNormal;
    i.colTime = colTime;
    i.lastValidPos1 = lastValidPos1;
    i.lastValidPos2 = lastValidPos2;
    i.penetration = penetration;
    i.relSpeed = relSpeed;
    return i;
}

__device__ inline IntersectionData make_intersectiondata()
{
    return make_intersectiondata(false, make_float3(0), 0.0f, make_float3(0), make_float3(0), 0, make_float3(0));
}

__device__ IntersectionData collideSpherePlane(Sphere* sphere, Plane* plane, float dt)
{
    Sphere updated = *sphere;
    updated.position += dt * updated.impulse;

    float distanceAfterFullUpdate = dot(plane->normal, updated.position) - plane->d;
    float penetration = abs(sphere->radius - distanceAfterFullUpdate);
    float3 relSpeed = updated.impulse / updated.mass;

    bool intersects = dot(plane->normal, updated.position) <= plane->d + updated.radius * length(plane->normal);
    if (!intersects)
    {
        // return empty
        return make_intersectiondata();
    }

    float3 colNormal = normalize(plane->normal);
    float colTime = dt * ((dot(plane->normal, sphere->position) - plane->d - sphere->radius * length(plane->normal)) / (dot(plane->normal, sphere->position) - dot(plane->normal, updated.position)));
    float3 lastValidPos = sphere->position + sphere->impulse * colTime;

    return make_intersectiondata(intersects, colNormal, colTime, lastValidPos, plane->center, penetration, relSpeed);
}

__device__ IntersectionData collideSphereSphere(Sphere* sphere1, Sphere *sphere2, float dt)
{
    Sphere s1 = *sphere1;
    Sphere s2 = *sphere2;

    float3 relSpeed = s2.impulse / s2.mass - s1.impulse / s1.mass;

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


    float distanceAfterFullUpdate = length((sphere1->position + dt * sphere1->impulse) - (sphere2->position + dt * sphere2->impulse));
    float penetration = sphere1->radius + sphere2->radius - distanceAfterFullUpdate;

    return make_intersectiondata(true, colNormal, colTime, lastValidPos1, lastValidPos2, penetration, relSpeed);
}

__device__ void resolveCollisionKinematically(Sphere* sphere, IntersectionData* intersection)
{
    sphere->impulse = reflect(sphere->impulse, intersection->colNormal);
    sphere->position = intersection->lastValidPos1;
}

__device__ void resolveCollisionDynamically(Sphere* sphere, IntersectionData* intersection, float dt)
{
    float lamda_spring = 10;
    float lamda_dashpot = .1;
    float lamda_shear = .1;

    float3 f_spring = lamda_spring * intersection->penetration * intersection->colNormal;
    float3 f_dashpot = lamda_dashpot * intersection->relSpeed;
    float3 f_shear = lamda_shear * (intersection->relSpeed - dot(intersection->relSpeed, intersection->colNormal) * intersection->colNormal);

    sphere->impulse += f_spring;
    sphere->position = intersection->lastValidPos1;
}

