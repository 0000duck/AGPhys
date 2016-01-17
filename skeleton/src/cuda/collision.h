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
    if (dot(sphere.velocity, plane.normal) > 0)
    {
        // moving away from plane
        //return -1.0f;
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

        if (length(sphere1.velocity - sphere2.velocity) < 0.001f)
        {
            return abs(penetration);
        }

        float3 colNormal = normalize(sphere2.position - sphere1.position);
        float s1_vAlongNormal = dot(sphere1.velocity, colNormal);
        float s2_vAlongNormal = dot(sphere2.velocity, -colNormal);

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

__device__ void kinematicCollisionResponseSpherePlane(Sphere& sphere, Plane& plane, float penetration)
{
    float3 colNormal = plane.normal;
    sphere.velocity = 0.9 * reflect(sphere.velocity, colNormal);
    sphere.position += colNormal * penetration;
}

__device__ void kinematicCollisionResponseSphereSphere(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere2.position - sphere1.position);
    sphere1.velocity = 0.9 * reflect(sphere1.velocity, colNormal);
    sphere2.velocity = 0.9 * reflect(sphere2.velocity, colNormal);

    sphere1.position -= colNormal * 0.5 * penetration;
    sphere2.position += colNormal * 0.5 * penetration;
}

__device__ void dynamicCollisionResponseSpherePlane(Sphere& sphere, Plane& plane, float penetration, float dt)
{
    float invMass = 1.0 / sphere.mass;
    float vNormal = -length(dot(sphere.velocity,plane.normal) * plane.normal);
    float epsilon = 0.7;
    float j = -(1+epsilon) / invMass *vNormal;


    float3 J = j * plane.normal;


    float l = length(sphere.velocity - (vNormal * plane.normal));
    if (l > 0){
        //friction
        float mu = 0.1;
        float3 frictionTerm = (sphere.velocity - (vNormal * plane.normal)) / l;

        J -= mu * j * frictionTerm;
    }

    const float deltaT = 1.0f;
    float3 momentum = J*deltaT;

    float3 toAdd = momentum * invMass;

    sphere.velocity += toAdd;

    sphere.force += momentum;
}

__device__ void dynamicCollisionResponseSphereSphere(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere2.position - sphere1.position);
    float3 vRel      = (dot(sphere1.velocity, colNormal) * colNormal) - (dot(sphere2.velocity, colNormal) * colNormal);

    float lamda_spring  = 1;
    float lamda_dashpot = 0;
    float lamda_shear   = 0.3;

    float3 f_spring  = lamda_spring * reflect(sphere1.velocity, colNormal);
    float3 f_dashpot = lamda_dashpot * vRel * sphere1.mass;
    float3 f_shear   = make_float3(0);

    sphere1.velocity = (f_spring + f_dashpot + f_shear);
    sphere2.velocity = -1.0f * (f_spring + f_dashpot + f_shear);

    sphere1.position -= colNormal * 0.5 * penetration;
    sphere2.position += colNormal * 0.5 * penetration;

}






/// ------------------------------------------------------ ELASTIC COLLISION RESPONSE  ------------------------------------------------------

__device__ void elasticCollision(Sphere& sphere1, Sphere& sphere2, float penetration)
{
    float3 colNormal = normalize(sphere1.position - sphere2.position);
    float3 _v1 = sphere1.velocity + (2 * dot(sphere2.velocity - sphere1.velocity, colNormal)) / (1 / sphere1.mass + 1 / sphere2.mass) * (1 / sphere1.mass) * colNormal;
    float3 _v2 = sphere2.velocity - (2 * dot(sphere2.velocity - sphere1.velocity, colNormal)) / (1 / sphere1.mass + 1 / sphere2.mass) * (1 / sphere2.mass) * colNormal;
    sphere1.velocity = _v1;
    sphere2.velocity = _v2;

    sphere1.position += colNormal * 0.5 * penetration;
    sphere2.position -= colNormal * 0.5 * penetration;
}


/// ---------------------------------------------------- RIGID BODY COLLISION RESPONSE  -----------------------------------------------------

__device__ void rigidBodyCollisionResponse()
{

}




