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

    int sphereCount = 100;
    int planeCount = 0;

    VertexBuffer<CUDA::Sphere> sphereBuffer;
    VertexBuffer<CUDA::Plane> planeBuffer;

    MVPShader* sphereShader;
    MVPShader* planeShader;


    CollisionSystem();
    ~CollisionSystem();

    void init();


    void update(float dt);
    void render(Camera *cam);

    void keyPressed(int key);
    void keyReleased(int key);

private:
    Interop	sphere_interop;
    Interop plane_interop;

};


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes();

