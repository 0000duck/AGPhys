#pragma once

#include <vector>
#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "cuda/particle.h"
#include "cuda/plane.h"
#include "cuda/interop.h"
#include "camera.h"
#include "particleSystem.h"
#include "collisionSystem.h"


class myRenderer
{
public:


    VertexBuffer<VertexN> grid_bottom;
    VertexBuffer<VertexN> grid_left;
    VertexBuffer<VertexN> grid_right;
    VertexBuffer<VertexN> grid_front;
    VertexBuffer<VertexN> grid_back;


    VertexBuffer<CUDA::Plane> planeBuffer;
    int planeCount;

    MVPColorShader* gridShader;
    ParticleSystem particleSystem;
    CollisionSystem collisionSystem;

    Interop plane_interop;

    myRenderer();
    ~myRenderer();

    void init();
    void shutdown();


    void update(float dt);
    void render(Camera *cam);


};


