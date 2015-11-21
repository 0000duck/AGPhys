#pragma once

#include "particle.h"

using namespace CUDA;

#ifdef __CUDACC__

class __align__(4) IntersectionData
{
public:
   bool intersects;
   
}__attribute__((packed));

#endif

__device__ inline IntersectionData make_intersectiondata(bool intersects)
{
	IntersectionData i;
	i.intersects = intersects;
	return i;
}

__device__ IntersectionData collideSpherePlane(Particle* sphere)
{
	return make_intersectiondata(sphere->position.x == 2);
}
