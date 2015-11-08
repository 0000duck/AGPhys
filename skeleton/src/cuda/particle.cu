#include <cstdlib>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <iostream>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

#include "particle.h"

using std::cout;
using std::endl;

namespace CUDA{


#define checked_cuda(ans) { gpu_assert((ans), __FILE__, __LINE__); }
inline void gpu_assert(cudaError_t code, char *file, int line, bool abort=true) {
    if (code != cudaSuccess) {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}

#define START_CUDA_TIMER cudaEvent_t start, stop;\
    cudaEventCreate(&start);cudaEventCreate(&stop);\
    cudaEventRecord(start);
#define STOP_CUDA_TIMER(TIME)  cudaEventRecord(stop);cudaEventSynchronize(stop);\
    cudaEventElapsedTime(&TIME, start, stop);\
    cudaEventDestroy(start); cudaEventDestroy(stop);



__device__ unsigned int get_global_index(void)
{
    unsigned int threadid = threadIdx.y * blockDim.x + threadIdx.x;
    unsigned int blockNumInGrid   = blockIdx.x  + gridDim.x  * blockIdx.y;
    unsigned int block_width =  blockDim.x * blockDim.y;
    unsigned int globalThreadNum = blockNumInGrid * block_width + threadid;
    return globalThreadNum;
}


__global__ void testKernel(float *g_idata, float *g_odata)
{
    int tid = threadIdx.x;

    // write data to global memory
    g_odata[tid] = g_idata[tid]+5;
}

void test()
{
    unsigned int num_threads = 32;
    unsigned int mem_size = sizeof(float) * num_threads;

    // allocate host memory
    float *h_idata = (float *) malloc(mem_size);

    // initalize the memory
    for (unsigned int i = 0; i < num_threads; ++i)
    {
        h_idata[i] = (float) i;
    }

    // allocate device memory
    float *d_idata;
    cudaMalloc((void **) &d_idata, mem_size);
    // copy host memory to device
    cudaMemcpy(d_idata, h_idata, mem_size,cudaMemcpyHostToDevice);

    // allocate device memory for result
    float *d_odata;
    cudaMalloc((void **) &d_odata, mem_size);

    // setup execution parameters
    dim3  grid(1, 1, 1);
    dim3  threads(num_threads, 1, 1);

    // execute the kernel
    testKernel<<< grid, threads, mem_size >>>(d_idata, d_odata);


    // allocate mem for the result on host side
    float *h_odata = (float *) malloc(mem_size);
    // copy result from device to host
    cudaMemcpy(h_odata, d_odata, sizeof(float) * num_threads,cudaMemcpyDeviceToHost);

    bool result = true;
    for (unsigned int i = 0; i < num_threads; ++i)
    {
        std::cout<<h_odata[i]<<std::endl;
        if(h_odata[i]!=i+5)
            result = false;
    }

    if(result){
        std::cout<<"CUDA test: SUCCESS!"<<std::endl;
    }else{
         std::cout<<"CUDA test: FAILED!"<<std::endl;
    }



    // cleanup memory
    free(h_idata);
    free(h_odata);
    cudaFree(d_idata);
    cudaFree(d_odata);

}

__device__ float generateRandomNumber(curandState* state, float min, float max)
{
	int tid = get_global_index();
	curandState localState = state[tid];
	float random = curand_uniform(&localState) * (max - min) + min;
	state[tid] = localState;
	return random;
}

__device__ float noise(curandState* state, float min, float max)
{
	return generateRandomNumber(state, min, max);
}

__device__ vec2_t noise2D(curandState* state, vec2_t min, vec2_t max)
{
	vec2_t result;
	result.x = generateRandomNumber(state, min.x, max.x);
	result.y = generateRandomNumber(state, min.y, max.y);
	return result;
}

__device__ vec3_t noise3D(curandState* state, vec3_t min, vec3_t max)
{
	vec3_t result;
	result.x = generateRandomNumber(state, min.x, max.x);
	result.y = generateRandomNumber(state, min.y, max.y);
	result.z = generateRandomNumber(state, min.z, max.z);
	return result;
}

__device__ vec3_t distractDirection3D(curandState* state, vec3_t origDirection, float maxAngle)
{
	float theta = noise(state, 0, maxAngle);
	float x = noise(state, 0, 0.99999999);
	float phi = 2 * 3.14159265359 * x;
	
	vec3_t yAxis = normalize(origDirection);
	vec3_t a; a.x = 1; a.y = 0; a.z = 0;
	if (yAxis.x == a.x && yAxis.y == a.y && yAxis.z == a.z)
	{
		a.x = 0; a.y = 1;
	}
	
	vec3_t xAxis = cross(yAxis, a);
	vec3_t zAxis = cross(xAxis, yAxis);
	
	
	vec3_t direction = sin(theta) * cos(phi) * zAxis + sin(theta) * sin(phi) * xAxis + cos(theta) * yAxis;
	return direction;
}

__global__ void resetParticles(Particle* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
	
	int tid = threadIdx.y * x * z + threadIdx.z * x + threadIdx.x;
	if (tid < numberOfParticles)
	{
		particles[tid].initial_position.x = threadIdx.x * distance + cornerX;
		particles[tid].initial_position.y = threadIdx.y * distance + cornerY;
		particles[tid].initial_position.z = threadIdx.z * distance + cornerZ;
		
		particles[tid].initial_color.x = 1.0;
		particles[tid].initial_color.y = 0.0;
		particles[tid].initial_color.z = 0.0;
		particles[tid].initial_color.w = 1.0;
		
		particles[tid].position = particles[tid].initial_position;
		particles[tid].color	= particles[tid].initial_color;
		
		particles[tid].impulse.x = 0;
		particles[tid].impulse.y = 0.1;
		particles[tid].impulse.z = 0;
		
		particles[tid].max_lifetime = 100;
		particles[tid].lifetime = particles[tid].max_lifetime;
	}
}

__device__ void integrateSingleParticle(Particle* particle, float dt, curandState* state)
{
	particle->position += dt * particle->impulse;
	particle->lifetime -= dt;
	particle->color.z = particle->lifetime / particle->max_lifetime;
	
	if (particle->lifetime < 0)
	{
		vec3_t min; min.x = 0; min.y = 0; min.z = 0;
		vec3_t max; max.x = 15; max.y = 15; max.z = 15;
		//particle->position 	= noise3D(state, min, max);
		particle->position 	= make_float3(0);
		particle->color 	= particle->initial_color;
		
		vec3_t impulse; impulse.x = 1; impulse.y = 1; impulse.z = 0;
		particle->impulse = distractDirection3D(state, impulse, 0.12);
		
		particle->max_lifetime 	= noise(state, 100, 115);
		particle->lifetime		= particle->max_lifetime;
	}
}

__global__ void integrateParticles(Particle* particles, float dt, curandState* state)
{
	int tid = threadIdx.x;
	integrateSingleParticle(&particles[tid], dt, state);
}

__global__ void setupRandomNumberState(curandState* state, unsigned long seed)
{
    int tid = threadIdx.x;
    curand_init(seed, tid, 0, &state[tid]);
} 

static curandState* buildStates(int numberOfStates)
{
	curandState* state;
	cudaMalloc((void **) &state, numberOfStates * sizeof(curandState));
	setupRandomNumberState<<<1, numberOfStates>>>(state, 1234);
	return state;
}

static void freeStates(curandState* state)
{
	cudaFree(state);
}

void resetParticles(void* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
	int y = numberOfParticles / (x * z);
	if (numberOfParticles % (x * z) > 0) y++;
    dim3 threads(x, y, z);
	resetParticles<<<1, threads>>>(static_cast<Particle*>(particles), numberOfParticles, x, z, cornerX, cornerY, cornerZ, distance);
}

void integrateParticles(void* particles, int numberOfParticles, float dt)
{
	curandState* state = buildStates(numberOfParticles);
	
    dim3 threads(numberOfParticles, 1, 1);
    integrateParticles<<<1, threads>>>(static_cast<Particle*>(particles), dt, state);
    
    freeStates(state);
}

}
