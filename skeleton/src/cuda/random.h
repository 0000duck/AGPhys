#pragma once

<<<<<<< HEAD
=======
#ifndef RANDOM_H
#define RANDOM_H

>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3
#include <ctime>
#include <curand.h>
#include <curand_kernel.h>
#include <cuda_runtime.h>
#include "helper_cuda.h"

curandState* random_states_host;
__device__ curandState* random_states_device;
bool initializedRandom = false;
int threadsPerBlock = 128;
__device__ int stateCount = 0;

__device__ float generateRandomNumber(float min, float max)
{
	int tid = (blockDim.x * blockIdx.x + threadIdx.x) % stateCount;
	curandState localState = random_states_device[tid];
	float random = curand_uniform(&localState) * (max - min) + min;
	random_states_device[tid] = localState;
	return random;
}

__device__ float noise(float min, float max)
{
	return generateRandomNumber(min, max);
}

__device__ float2 noise2D(float2 min, float2 max)
{
	float2 result;
	result.x = generateRandomNumber(min.x, max.x);
	result.y = generateRandomNumber(min.y, max.y);
	return result;
}

__device__ float3 noise3D(float3 min, float3 max)
{
	float3 result;
	result.x = generateRandomNumber(min.x, max.x);
	result.y = generateRandomNumber(min.y, max.y);
	result.z = generateRandomNumber(min.z, max.z);
	return result;
}

__device__ float3 distractDirection3D(float3 origDirection, float maxAngle)
{
	float theta = noise(0, maxAngle);
	float phi = noise(0, 2 * 3.14159265359);
	
	float3 yAxis = normalize(origDirection);
	float3 a; a.x = 1; a.y = 0; a.z = 0;
	if (yAxis.x == a.x && yAxis.y == a.y && yAxis.z == a.z)
	{
		a.x = 0; a.y = 1;
	}
	
	float3 xAxis = cross(yAxis, a);
	float3 zAxis = cross(xAxis, yAxis);
	
	
	float3 direction = sin(theta) * cos(phi) * zAxis + sin(theta) * sin(phi) * xAxis + cos(theta) * yAxis;
	return direction;
}

__global__ void setupRandomNumberState(curandState* states, int numberOfStates, unsigned long seed)
{
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfStates)
	{
		curand_init(seed, tid, 0, &states[tid]);
	}
}

__global__ void setDeviceVariables(curandState* states, int numberOfStates)
{
	random_states_device = states;
	stateCount = numberOfStates;
}

void initRandom(int numberOfStates)
{
	if (initializedRandom) return;
	
	unsigned long seed = std::time(NULL);
	cudaMalloc((void **) &random_states_host, numberOfStates * sizeof(curandState));
	
	int blocks = numberOfStates / threadsPerBlock + 1;
	
	setupRandomNumberState<<<blocks, threadsPerBlock>>>(random_states_host, numberOfStates, seed);
	setDeviceVariables<<<1, 1>>>(random_states_host, numberOfStates);
	initializedRandom = true;
}

void shutdownRandom()
{
	if (!initializedRandom)
	{ 
		return;
	}
	cudaFree(random_states_host);
	initializedRandom = false;
}
<<<<<<< HEAD
=======

#endif
>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3
