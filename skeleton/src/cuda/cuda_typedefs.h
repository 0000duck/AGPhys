#pragma once

#ifndef CUDA_TYPEDEFS_H
#define CUDA_TYPEDEFS_H

namespace CUDA{


#ifdef __CUDACC__

typedef float2 vec2_t;
typedef float3 vec3_t;
typedef float4 vec4_t;
typedef quaternion<float> quat_t;
#define ATTRIBUTES __attribute__((packed))
#define ALIGNMENT __align__(4)

#else

typedef vec2 vec2_t;
typedef vec3 vec3_t;
typedef vec4 vec4_t;
typedef glm::quat quat_t;
#define ATTRIBUTES
#define ALIGNMENT

#endif

}

#endif // CUDA_TYPEDEFS_H

