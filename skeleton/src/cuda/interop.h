#pragma once

#ifndef INTEROP_H
#define INTEROP_H

#include "helper_cuda.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>

class Interop
{
public:
	void 	registerGLBuffer(int glbuffer);
	void 	map();
	void 	unmap();
	void*	getDevicePtr();
	
private:	
	cudaGraphicsResource* resource;
};

#endif
