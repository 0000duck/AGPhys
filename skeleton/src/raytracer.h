#pragma once

#include "saiga/util/singleton.h"
#include "saiga/geometry/material_mesh.h"
#include "cuda/sphere.h"


class RayTracer : public Singleton<RayTracer>
{
    friend class Singleton<RayTracer>;

public:
    int fillBufferWithSpheres(MaterialMesh<VertexNT,GLuint>* input, std::vector<CUDA::Sphere>& output, float sphereRadius, float mass);
    int fillBufferWithSpheres(std::vector<CUDA::Sphere>& output, float sphereRadius, float mass);

private:

};
