#pragma once

#include "saiga/util/singleton.h"
#include "saiga/geometry/material_mesh.h"
#include "cuda/sphere.h"


class RayTracer : public Singleton<RayTracer>
{
    friend class Singleton<RayTracer>;

public:
    void fillBufferWithSpheres(MaterialMesh<VertexNT,GLuint>* input, std::vector<CUDA::Sphere>& output, float sphereRadius, float mass);
    void fillBufferWithSpheres(std::vector<CUDA::Sphere>& output, float sphereRadius, float mass);

private:

};
