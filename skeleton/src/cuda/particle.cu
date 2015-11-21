#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"
#include "random.h"


#include "random.h"

#include "particle.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * 3.14159265359 / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / 3.14159265359)

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

<<<<<<< HEAD
__global__ void resetParticlesGrid(Particle* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfParticles)
	{		
=======

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

__global__ void resetParticlesGrid(Particle* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfParticles)
	{
>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3
		int layerSize = x * z;
		int yPos = tid / layerSize;
		int normId = tid - yPos * layerSize;
		
		int xPos = normId % x;
		int zPos = (normId - xPos) / x;
		
		particles[tid].position.x = xPos * distance + cornerX;
		particles[tid].position.y = yPos * distance + cornerY;
		particles[tid].position.z = zPos * distance + cornerZ;
		
		particles[tid].color.x = 1.0;
		particles[tid].color.y = 0.0;
		particles[tid].color.z = 0.0;
	}
}

__global__ void resetParticlesVolcanoAndStar(Particle* particles, int numberOfParticles)
{
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfParticles)
	{
		particles[tid].lifetime = 0;
		particles[tid].color	= make_float4(1);
	}
}

__device__ void integrateParticle(Particle* p, float dt)
{
	p->lifetime    -= dt;
	p->position	   += p->impulse * dt;
	p->color.x 		= p->lifetime / p->max_lifetime;
	p->color.y		= 0.0;
	p->color.z		= 1.0 - (p->lifetime / p->max_lifetime);
}

__global__ void integrateParticlesStar(Particle* particles, int numberOfParticles, float dt)
{
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfParticles)
	{
		Particle* p 	= &particles[tid];
		integrateParticle(p, dt);
		
		if (p->lifetime < 0)
		{
			p->max_lifetime	= noise(7, 10);
			p->lifetime		= p->max_lifetime;
			
			p->position		= make_float3(0);
			p->impulse		= normalize(noise3D(make_float3(-1), make_float3(1)));
		}
	}
}

__global__ void integrateParticlesVolcano(Particle* particles, int numberOfParticles, float maxAngle, float dt)
{
	int tid = blockDim.x * blockIdx.x + threadIdx.x;
	if (tid < numberOfParticles)
	{
		Particle* p 	= &particles[tid];
		integrateParticle(p, dt);
		
		if (p->lifetime < 0)
		{
			p->max_lifetime	= noise(7, 10);
			p->lifetime		= p->max_lifetime;
			
			p->position		= make_float3(0);
			float3 dir = make_float3(0); dir.y = 1; // upwards
			p->impulse		= normalize(distractDirection3D(dir, maxAngle));
		}
	}
}
<<<<<<< HEAD
=======


static const int threadsPerBlock = 128;
>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3

void resetParticlesGrid(void* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
	int blocks = numberOfParticles / threadsPerBlock + 1;
	resetParticlesGrid<<<blocks, threadsPerBlock>>>(static_cast<Particle*>(particles), numberOfParticles, x, z, cornerX, cornerY, cornerZ, distance);
}

void resetParticlesVolcanoAndStar(void* particles, int numberOfParticles)
{
	int blocks = numberOfParticles / threadsPerBlock + 1;
	resetParticlesVolcanoAndStar<<<blocks, threadsPerBlock>>>(static_cast<Particle*>(particles), numberOfParticles);
}

void integrateParticlesStar(void* particles, int numberOfParticles, float dt)
{
	int blocks = numberOfParticles / threadsPerBlock + 1;
	initRandom(numberOfParticles);
	
	integrateParticlesStar<<<blocks, threadsPerBlock>>>(static_cast<Particle*>(particles), numberOfParticles, dt);
}

void integrateParticlesVolcano(void* particles, int numberOfParticles, float maxAngle, float dt)
{
	int blocks = numberOfParticles / threadsPerBlock + 1;
	initRandom(numberOfParticles);
	
	integrateParticlesVolcano<<<blocks, threadsPerBlock>>>(static_cast<Particle*>(particles), numberOfParticles, degreesToRadians(maxAngle), dt);
}

}
