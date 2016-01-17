#pragma once

//#define RIGIDBODY


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
#include "cuda/cloth.h"

class Cloth
{
public:
    void init();
    void shutdown();

    void update(float dt);
    void render(Camera* cam);


private:
    int numberOfSpheres;
    VertexBuffer<CUDA::Sphere> sphereBuffer;
    Interop	sphere_interop;

    MVPShader* sphereShader;

    CUDA::Cloth cudaCloth;
    int m, n;
};

class CollisionSystem : public SDL_KeyListener
{
private:
    int sphereCount = 1; // note: adapt the collision area for the linked cell algorithm when using a lot more spheres: collisionSystem.cpp line ~98
    float maxRadius = 0.0f;

    IndexedVertexBuffer<VertexNT,GLuint> teapot_buffer;

    VertexBuffer<CUDA::Sphere> body_sphere_buffer;
    Interop body_interop;
    std::vector<CUDA::RigidBody> bodies;


    VertexBuffer<CUDA::Sphere> sphereBuffer;
    Interop	sphere_interop;

    MVPShader* sphereShader;
    MVPShader* teapotShader;

#ifdef RIGIDBODY
    enum
    {
        TRIANGLE_MESH,
        SPHERES
    } method = SPHERES;
#else
    enum
    {
        BRUTE_FORCE,
        SORT_AND_SWEEP,
        LINKED_CELL
    } method = BRUTE_FORCE;
#endif

public:
    CollisionSystem();
    ~CollisionSystem();

    void init(int numberOfPlanes);
    void shutdown();

    void update(float dt, CUDA::Plane* planes, int planeCount);
    void render(Camera *cam);

    void reset();

    void keyPressed(int key);
    void keyReleased(int key);

};


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes();

