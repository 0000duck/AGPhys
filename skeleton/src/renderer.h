#pragma once


#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "cuda/particle.h"
#include "camera.h"
#include "particleSystem.h"


class myRenderer
{
public:


    VertexBuffer<VertexN> grid;
    MVPColorShader* gridShader;
    ParticleSystem particleSystem;

    myRenderer();
    ~myRenderer();

    void init();


    void update(float dt);
    void render(Camera *cam);


};


