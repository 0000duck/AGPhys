#pragma once


#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "saiga/geometry/material_mesh.h"
#include "saiga/opengl/indexedVertexBuffer.h"
#include "cuda/sphere.h"
#include "cuda/plane.h"
#include "cuda/rigidbody.h"
#include "camera.h"
#include "sdl/sdl_eventhandler.h"
#include "cuda/interop.h"

class CollisionSystem : public SDL_KeyListener
{
private:
    int sphereCount = 2000; // note: adapt the collision area for the linked cell algorithm when using a lot more spheres: collisionSystem.cpp line ~98
    float maxRadius = 0.0f;

    IndexedVertexBuffer<VertexNT,GLuint> teapot_buffer;

    VertexBuffer<CUDA::Sphere> body_sphere_buffer;
    Interop body_interop;
    std::vector<CUDA::RigidBody> bodies;


    VertexBuffer<CUDA::Sphere> sphereBuffer;
    Interop	sphere_interop;

    MVPShader* sphereShader;
    MVPShader* teapotShader;

    enum
    {
        BRUTE_FORCE,
        SORT_AND_SWEEP,
        LINKED_CELL
    } method = BRUTE_FORCE;

public:
    CollisionSystem();
    ~CollisionSystem();

    void init();
    void shutdown();

    void update(float dt, CUDA::Plane* planes, int planeCount);
    void render(Camera *cam);

    void reset();

    void keyPressed(int key);
    void keyReleased(int key);

};


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes();

