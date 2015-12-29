#define RIGIDBODY

#include "collisionSystem.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include "saiga/opengl/objloader.h"
#include "raytracer.h"
#include <random>
#include <ctime>

#include <glm/gtc/type_ptr.hpp>

#include "cuda/timing.h"


#define checked_cuda(ans) { gpu_assert((ans), __FILE__, __LINE__); }
inline void gpu_assert(cudaError_t code, char *file, int line, bool abort=true) {
    if (code != cudaSuccess) {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}


CollisionSystem::CollisionSystem()
{

}

CollisionSystem::~CollisionSystem()
{
}

void CollisionSystem::init(int numberOfPlanes)
{
    std::srand(std::time(NULL));
    sphereShader = ShaderLoader::instance()->load<MVPShader>("particles.glsl");


#ifndef RIGIDBODY
    //initialize spheres with some random values
    std::vector<CUDA::Sphere> spheres(sphereCount);


    for (int i = 0; i < sphereCount; ++i)
    {
        CUDA::Sphere& p = spheres[i];
        p.position = vec3(0, 0.5f, 0); // doesn't matter as the position will be initialized in the reset function
        p.radius = 0.5f;
        //p.radius = ((std::rand() % 40) + 10) / 100.f; // 0.1 - 0.5 radius
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
        p.velocity = glm::ballRand(2.0f);
        p.mass = 1;

        p.id = i;
        p.nextInList = -1;

        if (p.radius > maxRadius)
        {
            maxRadius = p.radius;
        }
    }

    //upload sphere array to opengl
    sphereBuffer.set(spheres);
    sphereBuffer.setDrawMode(GL_POINTS);

    sphere_interop.registerGLBuffer(sphereBuffer.getVBO());

#else

    ObjLoader* loader = ObjLoader::instance();
    RayTracer* raytracer = RayTracer::instance();


    // load teapot
    MaterialMesh<VertexNT,GLuint>* teapot_mesh;
    teapot_mesh = loader->loadFromFile("objs/teapot.obj", NoParams());
    teapot_mesh->createBuffers(teapot_buffer);

    // create shader
    teapotShader = ShaderLoader::instance()->load<MVPShader>("object.glsl");

    // sphere buffer
    std::vector<CUDA::Sphere> spheres;
    //int count = raytracer->fillBufferWithSpheres(teapot_mesh, spheres, 0.02f, 100.f);
    int count = raytracer->fillBufferWithSpheres(spheres, 0.5f, 100.f);

    // load to vertex buffer
    body_sphere_buffer.set(spheres);
    body_sphere_buffer.setDrawMode(GL_POINTS);


    // rigid body buffer
    bodies.push_back(CUDA::RigidBody());
    bodies[0].linearVelocity = vec3(0.f);
    bodies[0].angularVelocity = vec3(0.f, 0.f, 0.f);
    bodies[0].mass = 100.0f;
    bodies[0].numberOfSpheres = count;
    bodies[0].position = vec3(0.0f, 7.0f, 0.0f);
    bodies[0].rotation = quat();
    bodies[0].torque = vec3(8.f, 0.f, 0.f);

    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            bodies[0].invInertia[y][x] = 0.f;
            if (x == y)
            {
                bodies[0].invInertia[y][x] = 1.f/150.f;
            }
        }
    }


    // interops
    body_interop.registerGLBuffer(body_sphere_buffer.getVBO());
    sphereCount = spheres.size();

    CUDA::initRigidBodies(&bodies[0], bodies.size(), numberOfPlanes);

#endif

    initTiming();

    reset();
}

void CollisionSystem::shutdown()
{
    shutdownTiming();
#ifdef RIGIDBODY
    CUDA::shutdownRigidBodies();
#endif
}

void CollisionSystem::reset()
{
#ifndef RIGIDBODY
    sphere_interop.map();
    void* spheres_ptr = sphere_interop.getDevicePtr();
    CUDA::resetSpheres(static_cast<CUDA::Sphere*>(spheres_ptr), sphereCount, 9, 9, glm::vec3(-8.f, 1.f, -8.f), 2);
    sphere_interop.unmap();
#endif
}

void CollisionSystem::update(float dt, CUDA::Plane* planes, int planeCount)
{
#ifndef RIGIDBODY
    sphere_interop.map();
    void* spheres = sphere_interop.getDevicePtr();

    glm::vec3 colArea(18.f, 50.f, 18.f); // for linked cell. declares the area to partition into cell grid

    switch (method)
    {
        case BRUTE_FORCE:
            CUDA::updateAllSpheresBruteForce(static_cast<CUDA::Sphere*>(spheres), static_cast<CUDA::Plane*>(planes), sphereCount, planeCount, dt);
            break;

        case SORT_AND_SWEEP:
            CUDA::updateAllSpheresSortAndSweep(static_cast<CUDA::Sphere*>(spheres), static_cast<CUDA::Plane*>(planes), sphereCount, planeCount, dt);
            break;

        case LINKED_CELL:
            CUDA::updateAllSpheresLinkedCell(static_cast<CUDA::Sphere*>(spheres), static_cast<CUDA::Plane*>(planes), sphereCount, planeCount, dt, colArea, glm::vec3(-9.f, -1.f, -9.f), maxRadius);
            break;
    }

    sphere_interop.unmap();
#else

    body_interop.map();
    CUDA::updateRigidBodies(static_cast<CUDA::Sphere*>(body_interop.getDevicePtr()), sphereCount, planes, planeCount, dt);
    body_interop.unmap();

#endif
}

void CollisionSystem::render(Camera *cam)
{
    //render the particles from the viewpoint of the camera
    sphereShader->bind();
#ifndef RIGIDBODY
    sphereShader->uploadAll(mat4(),cam->view,cam->proj);
    sphereBuffer.bindAndDraw();
#else
    std::vector<glm::vec3> pos(bodies.size());
    std::vector<glm::quat> rot(bodies.size());

    CUDA::getOrientationData(pos, rot);

    for (int i = 0; i < bodies.size(); ++i)
    {
        mat4 m;
        m = glm::translate(m, pos[i]);
        m = m * glm::mat4_cast(rot[i]);

        //std::cout << pos[i].x << " " << pos[i].y << " " << pos[i].z << std::endl;
        //std::cout << rot[i].x << " " << rot[i].y << " " << rot[i].z << " " << rot[i].w << std::endl;

        sphereShader->uploadAll(m,cam->view,cam->proj);
        body_sphere_buffer.bindAndDraw();

        teapotShader->bind();
        teapotShader->uploadAll(m, cam->view, cam->proj);
        //teapot_buffer.bindAndDraw();
        teapotShader->unbind();
    }

#endif
}

void CollisionSystem::keyPressed(int key)
{
    switch(key)
    {
        case SDLK_r:
            reset();
            break;

        case SDLK_1:
            method = BRUTE_FORCE;
            std::cout << "\n Switched to brute-force method" << std::endl;
            reset();
            break;

        case SDLK_2:
            method = SORT_AND_SWEEP;
            std::cout << "\n Switched to sort and sweep method" << std::endl;
            reset();
            break;

        case SDLK_3:
            method = LINKED_CELL;
            std::cout << "\n Switched to linked cell method" << std::endl;
            reset();
            break;
    }
}

void CollisionSystem::keyReleased(int key)
{

}


template<>
void VertexBuffer<CUDA::Sphere>::setVertexAttributes()
{
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glEnableVertexAttribArray( 2 );

    //position
    glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Sphere), NULL );
    //radius
    glVertexAttribPointer(1,1, GL_FLOAT, GL_FALSE, sizeof(CUDA::Sphere), (void*) (3 * sizeof(GLfloat)) );
    //color
    glVertexAttribPointer(2,4, GL_FLOAT, GL_FALSE, sizeof(CUDA::Sphere), (void*) (4 * sizeof(GLfloat)) );

}



