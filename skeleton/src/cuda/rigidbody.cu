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

namespace CUDA
{

RigidBody* dev_ptr;
int numberOfBodies;

void initRigidBodies(RigidBody* host_bodies, int size)
{
    cudaMalloc(&dev_ptr, sizeof(RigidBody) * size);
    cudaMemcpy(dev_ptr, host_bodies, sizeof(RigidBody) * size, cudaMemcpyHostToDevice);
    numberOfBodies = size;
}

void shutdownRigidBodies()
{
    cudaFree(dev_ptr);
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

    getPosAndRot<<<blocks, threadsPerBlock>>>(dev_ptr, pos_ptr, rot_ptr, numberOfBodies);

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
            m[x][y] *= s;
        }
    }
}

__host__ __device__ void matTimesVec(float m[3][3], vec3_t& in, vec3_t& out)
{
    out.x = m[0][0] * in.x + m[0][1] * in.y + m[0][2] * in.z;
    out.y = m[1][0] * in.x + m[1][1] * in.y + m[1][2] * in.z;
    out.z = m[2][0] * in.x + m[2][1] * in.y + m[2][2] * in.z;
}

__global__ void updateBodies(RigidBody* bodies, int numberOfBodies, Sphere* spheres, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfBodies)
    {
        RigidBody& rb = bodies[tid];

        uint sphereStart = 0;
        uint sphereEnd = 0;
        for (int i = 0; i < tid; ++i)
        {
            sphereStart += bodies[i].numberOfSpheres;
        }
        sphereEnd = sphereStart + rb.numberOfSpheres;

        rb.linearVelocity += dt * make_float3(0.0, -0.1f, 0.0f); // gravity
        //rb.position += dt * rb.linearVelocity;

        float rot[3][3];
        quatToRot3(rb.rotation, rot);
        float invRot[3][3];
        transposeMatrix(rot, invRot);


        // only for cube:
        float curInertia[3][3];
        matTimesScalar(rot, 1.f/60.f);
        matTimesMat(rot, invRot, curInertia);

        if (length(rb.torque) == 0.f)
            return;

        // equation 5
        matTimesVec(curInertia, rb.torque, rb.angularVelocity);
        
        // equation 7
        float3 rotationAxis = normalize(rb.angularVelocity);
        float rotationAngle = length(rb.angularVelocity * dt);
        quaternion<float> dq(rotationAxis * sin(rotationAngle / 2), cos(rotationAngle / 2));

        // equation 8
        rb.rotation = dq * rb.rotation;
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
            std::cout << m[x][y] << "    ";
        }
        std::cout << std::endl;
    }
}

void printDelim()
{
    std::cout << "-----------" << std::endl;
}

void updateRigidBodies(Sphere* spheres, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfBodies / threadsPerBlock + 1;

    updateBodies<<<blocks, threadsPerBlock>>>(dev_ptr, numberOfBodies, spheres, dt);

    float m1[3][3];
    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            m1[x][y] = y * 3 + x;
        }
    }
    printMat(m1);
    vec3_t v;
    v.x = 1.f; v.y = 2.f; v.z = 3.f;
    printVec(v);
    vec3_t r;
    matTimesVec(m1, v, r);
    printVec(r);
}



}
