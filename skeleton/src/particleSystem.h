#pragma once


#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "cuda/particle.h"
#include "camera.h"
#include "sdl/sdl_eventhandler.h"
#include "cuda/interop.h"

class ParticleSystem : public SDL_KeyListener
{
public:

    int particleCount = 100000;
    VertexBuffer<CUDA::Particle> particleBuffer;
    MVPShader* particleShader;


    ParticleSystem();
    ~ParticleSystem();

    void init();


    void update(float dt);
    void render(Camera *cam);

    void keyPressed(int key);
    void keyReleased(int key);

private:
	Interop	interop;

};


template<>
void VertexBuffer<CUDA::Particle>::setVertexAttributes();

