#pragma once


#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "cuda/sphere.h"
#include "cuda/plane.h"
#include "camera.h"
#include "sdl/sdl_eventhandler.h"
#include "cuda/interop.h"

class CollisionSystem : public SDL_KeyListener
{
public:

    int sphereCount = 150;

    VertexBuffer<CUDA::Sphere> sphereBuffer;

    MVPShader* sphereShader;


    CollisionSystem();
    ~CollisionSystem();

    void init();


    void update(float dt, CUDA::Plane* planes, int planeCount);
    void render(Camera *cam);

    void keyPressed(int key);
    void keyReleased(int key);

private:
    Interop	sphere_interop;

};


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes();

