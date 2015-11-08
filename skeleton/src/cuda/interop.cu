#include "interop.h"


namespace 
{
#define checked_cuda(ans) { gpu_assert((ans), __FILE__, __LINE__); }
inline void gpu_assert(cudaError_t code, char *file, int line, bool abort=true) {
    if (code != cudaSuccess) {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}
}

void Interop::registerGLBuffer(int glbuffer)
{
	cudaError_t err = cudaGraphicsGLRegisterBuffer(&resource, glbuffer, cudaGraphicsRegisterFlagsNone);
	checked_cuda(err);
}

void Interop::map()
{
	cudaError_t err = cudaGraphicsMapResources(1, &resource);
	checked_cuda(err);
}

void Interop::unmap()
{
	cudaError_t err = cudaGraphicsUnmapResources(1, &resource);
	checked_cuda(err);
}

void* Interop::getDevicePtr()
{
	void* ptr;
	size_t size;
	cudaError_t err = cudaGraphicsResourceGetMappedPointer(&ptr, &size, resource);
	checked_cuda(err);
	return ptr;
}
