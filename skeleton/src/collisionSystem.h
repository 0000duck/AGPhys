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
private:
    int sphereCount = 2000;
    float maxRadius = 0.0f;

    VertexBuffer<CUDA::Sphere> sphereBuffer;
    Interop	sphere_interop;

    MVPShader* sphereShader;

    enum
    {
        BRUTE_FORCE,
        SORT_AND_SWEEP,
        LINKED_CELL
    } method;

public:
    CollisionSystem();
    ~CollisionSystem();

    void init();


    void update(float dt, CUDA::Plane* planes, int planeCount);
    void render(Camera *cam);

    void reset();

    void keyPressed(int key);
    void keyReleased(int key);

};


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes();

