#pragma once

#include "sphere.h"
#include "plane.h"
#include "helper_math.h"
#include "helper_cuda.h"

using namespace CUDA;


/// ------------------------------------------------------ COLLISION DETECTION ------------------------------------------------------

/*
 * returns by how much the sphere intersects the plane and -1 if no collision
 */
__device__ float collideSpherePlane(Sphere& sphere, Plane& plane)
{
    if (dot(sphere.impulse, plane.normal) > 0)
    {
        // moving away from plane
        return -1.0f;
    }
    if (dot(plane.normal, sphere.position) <= plane.d + sphere.radius * length(plane.normal))
    {
        // intersects
        float penetration = dot(plane.normal, sphere.position) - plane.d - sphere.radius;
        return abs(penetration);
    }
    return -1.0f;
}

/*
 * returns by how much the spheres intersects each other and -1 if no collision
 */
__device__ float collideSphereSphere(Sphere& sphere1, Sphere& sphere2)
{
    if (length(sphere1.position - sphere2.position) <= sphere1.radius + sphere2.radius)
    {
        float penetration = sphere1.radius + sphere2.radius - length(sphere1.position - sphere2.position);

        if (length(sphere1.impulse - sphere2.impulse) < 0.001f)
        {
            return abs(penetration);
        }

        float3 colNormal = normalize(sphere2.position - sphere1.position);
        float s1_vAlongNormal = dot(sphere1.impulse, colNormal);
        float s2_vAlongNormal = dot(sphere2.impulse, -colNormal);

        if (s1_vAlongNormal >= 0.0f && s2_vAlongNormal >= 0.0f)
        {
            // both move towards collision point
            return abs(penetration);
        }

        if (s1_vAlongNormal < 0.0f)
        {
            // sphere1 moves away from collision point
            if (s2_vAlongNormal > abs(s1_vAlongNormal))
            {
                // s2 catches up
                return abs(penetration);
            }
            else {
                return -1.0f;
            }
        }
        if (s2_vAlongNormal < 0.0f)
        {
            // same for s2
            if (s1_vAlongNormal > abs(s2_vAlongNormal))
            {
                // s2 catches up
                return abs(penetration);
            }
            else {
                return -1.0f;
            }
        }


        return abs(penetration);
    }
    return -1.0f;
}




/// ------------------------------------------------------ COLLISION RESPONSE  ------------------------------------------------------

/*
 *
 */
__device__ void kinematicCollisionResponseSpherePlane(Sphere& sphere, Plane& plane, float penetration)
{
    float3 colNormal = plane.normal;
    sphere.impulse = 0.9 * reflect(sphere.impulse, colNormal);
    sphere.position += colNormal * penetration;
}

__device__ void kinematicCollisionResponseSphereSphere(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere2.position - sphere1.position);
    sphere1.impulse = 0.9 * reflect(sphere1.impulse, colNormal);
    sphere2.impulse = 0.9 * reflect(sphere2.impulse, colNormal);

    sphere1.position -= colNormal * 0.5 * penetration;
    sphere2.position += colNormal * 0.5 * penetration;
}

__device__ void dynamicCollisionResponseSpherePlane(Sphere& sphere, Plane& plane, float penetration)
{
    float3 colNormal = plane.normal;
    float3 vRel      = dot(sphere.impulse, plane.normal) * plane.normal;

    float lamda_spring  = 1;
    float lamda_dashpot = 0.3;
    float lamda_shear   = 0.3;

    float3 f_spring  = lamda_spring * reflect(sphere.impulse, colNormal);
    float3 f_dashpot = lamda_dashpot * vRel * sphere.mass;
    float3 f_shear   = lamda_shear * -1.0f * (sphere.impulse - vRel) * sphere.mass;

    sphere.impulse = (f_spring + f_dashpot + f_shear);
    sphere.position += colNormal * penetration;
}

__device__ void dynamicCollisionResponseSphereSphere(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere2.position - sphere1.position);
    float3 vRel      = (dot(sphere1.impulse, colNormal) * colNormal) - (dot(sphere2.impulse, colNormal) * colNormal);

    float lamda_spring  = 1;
    float lamda_dashpot = 0;
    float lamda_shear   = 0.3;

    float3 f_spring  = lamda_spring * reflect(sphere1.impulse, colNormal);
    float3 f_dashpot = lamda_dashpot * vRel * sphere1.mass;
    float3 f_shear   = make_float3(0);

    sphere1.impulse = (f_spring + f_dashpot + f_shear);
    sphere2.impulse = -1.0f * (f_spring + f_dashpot + f_shear);

    sphere1.position -= colNormal * 0.5 * penetration;
    sphere2.position += colNormal * 0.5 * penetration;

}






/// ------------------------------------------------------ ELASTIC COLLISION RESPONSE  ------------------------------------------------------

__device__ void elasticCollision(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere1.position - sphere2.position);
    float3 _v1 = sphere1.impulse + (2 * dot(sphere2.impulse - sphere1.impulse, colNormal)) / (1 / sphere1.mass + 1 / sphere2.mass) * (1 / sphere1.mass) * colNormal;
    float3 _v2 = sphere2.impulse - (2 * dot(sphere2.impulse - sphere1.impulse, colNormal)) / (1 / sphere1.mass + 1 / sphere2.mass) * (1 / sphere2.mass) * colNormal;
    sphere1.impulse = _v1;
    sphere2.impulse = _v2;

    sphere1.position += colNormal * 0.5 * penetration;
    sphere2.position -= colNormal * 0.5 * penetration;
}







