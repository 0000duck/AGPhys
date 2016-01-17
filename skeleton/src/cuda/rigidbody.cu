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
#include <thrust/reduce.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>


#include "rigidbody.h"
#include "collision.h"

namespace CUDA
{

RigidBody* body_ptr;
int* grid_ptr;
int numberOfBodies;
int numberOfPlanes;

void initRigidBodies(RigidBody* host_bodies, int size, int planeCount)
{
    cudaMalloc(&body_ptr, sizeof(RigidBody) * size);
    cudaMemcpy(body_ptr, host_bodies, sizeof(RigidBody) * size, cudaMemcpyHostToDevice);
    numberOfBodies = size;
    numberOfPlanes = planeCount;

    // init grid
    int gridSize = numberOfBodies * (numberOfBodies + numberOfPlanes);
    cudaMalloc(&grid_ptr, sizeof(int) * gridSize);
}

void shutdownRigidBodies()
{
    cudaFree(body_ptr);
    cudaFree(grid_ptr);
}

__global__ void getPosAndRot(RigidBody* bodies, vec3_t* pos_ptr, quat_t* rot_ptr, int numberOfBodies)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfBodies)
    {
        RigidBody& rb = bodies[tid];
        pos_ptr[tid] = rb.position;
        rot_ptr[tid] = rb.rotation;
    }
}

void getOrientationData(std::vector<glm::vec3>& pos, std::vector<glm::quat>& rot)
{
    int threadsPerBlock = 128;
    int blocks = numberOfBodies / threadsPerBlock + 1;

    vec3_t* pos_ptr;
    quat_t* rot_ptr;

    cudaMalloc(&pos_ptr, sizeof(vec3_t) * numberOfBodies);
    cudaMalloc(&rot_ptr, sizeof(quat_t) * numberOfBodies);

    getPosAndRot<<<blocks, threadsPerBlock>>>(body_ptr, pos_ptr, rot_ptr, numberOfBodies);

    cudaMemcpy(&pos[0], pos_ptr, sizeof(vec3_t) * numberOfBodies, cudaMemcpyDeviceToHost);
    cudaMemcpy(&rot[0], rot_ptr, sizeof(quat_t) * numberOfBodies, cudaMemcpyDeviceToHost);

    cudaFree(pos_ptr);
    cudaFree(rot_ptr);
}

__host__ __device__ void quatToRot3(quat_t& quat, float rot[3][3])
{
    rot[0][0] = 1.f - 2.f * quat.y * quat.y - 2.f * quat.z * quat.z;
    rot[0][1] = 2.f * quat.x * quat.y - 2.f * quat.w * quat.z;
    rot[0][2] = 2.f * quat.x * quat.z + 2.f * quat.w * quat.y;

    rot[1][0] = 2.f * quat.x * quat.y + 2.f * quat.w * quat.z;
    rot[1][1] = 1.f - 2.f * quat.x * quat.x - 2.f * quat.z * quat.z;
    rot[1][2] = 2.f * quat.y * quat.z - 2.f * quat.w * quat.x;

    rot[2][0] = 2.f * quat.x * quat.z - 2.f * quat.w * quat.z;
    rot[2][1] = 2.f * quat.y * quat.z + 2.f * quat.w * quat.x;
    rot[2][2] = 1.f - 2.f * quat.x * quat.x - 2.f * quat.y * quat.y;
}

__host__ __device__ void transposeMatrix(float in[3][3], float out[3][3])
{
    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            out[x][y] = in[y][x];
        }
    }
}

__host__ __device__ void matTimesMat(float l[3][3], float r[3][3], float out[3][3])
{
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            float sum = 0.f;
            for (int k = 0; k < 3; ++k)
            {
                sum += l[row][k] * r[k][col];
            }
            out[row][col] = sum;
        }
    }
}

__host__ __device__ void matTimesScalar(float m[3][3], float s)
{
    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            m[y][x] *= s;
        }
    }
}

__host__ __device__ void matTimesVec(float m[3][3], vec3_t& in, vec3_t& out)
{
    out.x = m[0][0] * in.x + m[0][1] * in.y + m[0][2] * in.z;
    out.y = m[1][0] * in.x + m[1][1] * in.y + m[1][2] * in.z;
    out.z = m[2][0] * in.x + m[2][1] * in.y + m[2][2] * in.z;
}

__host__ __device__ void quatTimesQuat(quat_t& l, quat_t& r, quat_t& out)
{
    float3 v0 = make_float3(l.x, l.y, l.z);
    float3 v1 = make_float3(r.x, r.y, r.z);

    out.w = l.w * r.w - dot(v0, v1);
    float3 v = l.w * v1 + r.w * v0 + cross(v0, v1);
    out.x = v.x;
    out.y = v.y;
    out.z = v.z;
}

__global__ void updateBodies(RigidBody* bodies, int numberOfBodies, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfBodies)
    {
        RigidBody& rb = bodies[tid];

        rb.position += (rb.linearVelocity / rb.mass) * dt;
        rb.linearVelocity += dt * make_float3(0.0, -1.5f, 0.0f); // gravity


        if (length(rb.angularMomentum) == 0.f)
            return;

        float rot[3][3];
        quatToRot3(rb.rotation, rot);
        float invRot[3][3];
        transposeMatrix(rot, invRot);

        // equation 6
        float curInertia[3][3];
        float tmp[3][3];
        matTimesMat(rot, rb.invInertia, tmp);
        matTimesMat(tmp, invRot, curInertia);

        // equation 5
        matTimesVec(curInertia, rb.angularMomentum, rb.angularVelocity);
        
        // equation 7
        float3 rotationAxis = normalize(rb.angularVelocity);
        float rotationAngle = length(rb.angularVelocity * dt);
        quaternion<float> dq(rotationAxis, rotationAngle);

        // equation 8
        quaternion<float> newRot = dq * rb.rotation;
        rb.rotation = newRot;
    }
}

__device__ inline void incrementGrid(int* grid, int width, int ownID, int otherID)
{
    int index = width * ownID + otherID;
    atomicAdd(&grid[index], 1);
}

__device__ inline void getAbsPositionAndVelocity(RigidBody& rb, Sphere& sphere, float3& pos, float3& vel)
{
    float3 abs_pos;
    float rot[3][3];
    quatToRot3(rb.rotation, rot);
    matTimesVec(rot, sphere.position, abs_pos);
    float3 ang = cross(rb.angularVelocity, abs_pos);
    float3 lin = rb.linearVelocity;
    vel = ang + lin;
    pos = abs_pos + rb.position;
}

__global__ void collisionDetection(RigidBody* bodies, int numberOfBodies, Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, int* grid)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        int rb_ID = 0;
        int gridWidth = numberOfBodies + numberOfPlanes;

        uint sphereSum = 0;
        for (int r = 0; r < numberOfBodies; ++r)
        {
            sphereSum += bodies[r].numberOfSpheres;
            if (tid < sphereSum)
            {
                // found corresponding body
                rb_ID = r;
                break;
            }
        }


        Sphere& sphere = spheres[tid];
        RigidBody& rb = bodies[rb_ID];

        sphere.force = make_float3(0.0);

        // absolute position & velocity
        float3 abs_pos, abs_vel;
        getAbsPositionAndVelocity(rb, sphere, abs_pos, abs_vel);

        Sphere s = sphere;
        s.position = abs_pos;
        s.velocity = abs_vel;

        sphere.sphereCollider = -1;
        sphere.planeCollider  = -1;

        // PLANE COLLISION
        for (int p = 0; p < numberOfPlanes; ++p)
        {
            float penetration = collideSpherePlane(s, planes[p]);
            if (penetration != -1.0f)
            {
                incrementGrid(grid, gridWidth, rb_ID, numberOfBodies + p);
                sphere.planeCollider = p;
                rb.linearVelocity = make_float3(0.f);
                rb.angularMomentum = make_float3(0.f);
            }
        }

        // SPHERE COLLISION - brute force for now
        for (int s = 0; s < numberOfSpheres; ++s)
        {

        }




    }
}

__device__ inline void atomicAddAngularMomentum(RigidBody& rb, const float3& t)
{
    atomicAdd(&rb.angularMomentum.x, t.x);
    atomicAdd(&rb.angularMomentum.y, t.y);
    atomicAdd(&rb.angularMomentum.z, t.z);
}

__device__ inline void atomicAddLinearVelocity(RigidBody& rb, const float3& l)
{
    atomicAdd(&rb.linearVelocity.x, l.x);
    atomicAdd(&rb.linearVelocity.y, l.y);
    atomicAdd(&rb.linearVelocity.z, l.z);
}

__global__ void collisionResponse(RigidBody* bodies, int numberOfBodies, Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, int* grid, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        int rb_ID = 0;
        int gridWidth = numberOfBodies + numberOfPlanes;

        uint sphereSum = 0;
        for (int r = 0; r < numberOfBodies; ++r)
        {
            sphereSum += bodies[r].numberOfSpheres;
            if (tid < sphereSum)
            {
                rb_ID = r;
                break;
            }
        }

        Sphere& sphere = spheres[tid];
        RigidBody& rb = bodies[rb_ID];

        // absolute position & velocity
        float3 abs_pos, abs_vel;
        getAbsPositionAndVelocity(rb, sphere, abs_pos, abs_vel);

        if (sphere.planeCollider != -1)
        {
            // PLANE RESPONSE

            Plane& plane = planes[sphere.planeCollider];
            int numberOfCollisions = grid[rb_ID * gridWidth + numberOfBodies + sphere.planeCollider];
            if (numberOfCollisions != 0)
            {
                float mass = rb.mass / numberOfCollisions;
                float vNormal = -length(dot(abs_vel,plane.normal) * plane.normal);
                float epsilon = 0.1;
                float j = -(1+epsilon) * mass *vNormal;



                float3 J = j * plane.normal;


                float l = length(abs_vel - (vNormal * plane.normal));
                if (l > 0){
                    //friction
                    float mu = 0.1;
                    float3 frictionTerm = (abs_vel - (vNormal * plane.normal)) / l;

                    J -= mu * j * frictionTerm;

                }

                const float deltaT = 1.0f;
                float3 momentum = J*deltaT;

                float3 toAdd = momentum ;

    //            particle.velocity += toAdd;

                sphere.force += toAdd;
            }
        }

        if (sphere.sphereCollider != -1)
        {
            // SPHERE RESPONSE
        }

        float3 linearForce = sphere.force;
        float3 torque = 0.1*cross(sphere.position, linearForce);


        //TODO use reduce
        atomicAddLinearVelocity(rb, linearForce);
        atomicAddAngularMomentum(rb, torque);

    }
}

__global__ void clearGrid(int* grid, int sizeOfGrid)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < sizeOfGrid)
    {
        grid[tid] = 0;
    }
}

void printVec(vec3_t& v)
{
    std::cout << "Vector: " << v.x << " " << v.y << " " << v.z << std::endl;
}

void printQuat(quat_t& q)
{
    std::cout << "Quaternion: " << q.x << " " << q.y << " " << q.z << " " << q.w << std::endl;
}

void printMat(float m[3][3])
{
    std::cout << "Matrix: " << std::endl;
    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            std::cout << m[y][x] << "    ";
        }
        std::cout << std::endl;
    }
}

void printDelim()
{
    std::cout << "-----------" << std::endl;
}

void updateRigidBodies(Sphere* spheres, int numberOfSpheres, Plane* planes, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfBodies / threadsPerBlock + 1;
    updateBodies<<<blocks, threadsPerBlock>>>(body_ptr, numberOfBodies, dt);
    blocks = (numberOfBodies * (numberOfBodies + numberOfPlanes)) / threadsPerBlock + 1;
    clearGrid<<<blocks, threadsPerBlock>>>(grid_ptr, numberOfBodies * (numberOfBodies + numberOfPlanes));
    blocks = numberOfSpheres / threadsPerBlock + 1;
    collisionDetection<<<blocks, threadsPerBlock>>>(body_ptr, numberOfBodies, spheres, numberOfSpheres, planes, numberOfPlanes, grid_ptr);
    collisionResponse<<<blocks, threadsPerBlock>>>(body_ptr, numberOfBodies, spheres, numberOfSpheres, planes, numberOfPlanes, grid_ptr, dt);

}



}
