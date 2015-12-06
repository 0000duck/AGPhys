#include "timing.h"


#include <curand_kernel.h>
#include <cuda_runtime.h>
#include "helper_math.h"
#include "helper_cuda.h"

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

static cudaEvent_t start = NULL;
static cudaEvent_t stop = NULL;
static bool initialized = false;

void initTiming()
{
    if (initialized)
        return;

    cudaError_t err = cudaEventCreate(&start);
    checked_cuda(err);

    err = cudaEventCreate(&stop);
    checked_cuda(err);

    initialized = true;
}

void shutdownTiming()
{
    if (!initialized)
        return;

    cudaError_t err = cudaEventDestroy(start);
    checked_cuda(err);

    err = cudaEventDestroy(stop);
    checked_cuda(err);

    initialized = false;
}

void startTiming()
{
    cudaError_t err = cudaEventRecord(start);
    checked_cuda(err);
}

float endTiming()
{
    cudaError_t err = cudaEventRecord(stop);
    checked_cuda(err);

    cudaEventSynchronize(stop);
    checked_cuda(err);

    float time = 0.0f;
    err = cudaEventElapsedTime(&time, start, stop);
    checked_cuda(err);

    return time;
}
