#include "collisionSystem.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include <random>
#include <ctime>


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

void CollisionSystem::init()
{
    std::srand(std::time(NULL));
    sphereShader = ShaderLoader::instance()->load<MVPShader>("particles.glsl");

    //initialize spheres with some random values
    std::vector<CUDA::Sphere> spheres(sphereCount);


    for (int i = 0; i < sphereCount; ++i)
    {
        CUDA::Sphere& p = spheres[i];
        p.position = vec3(0, 0.5f, 0); // doesn't matter as the position will be initialized in the reset function
        p.radius = 0.5f;
        //p.radius = ((std::rand() % 40) + 10) / 100.f; // 0.1 - 0.5 radius
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
        p.impulse = glm::ballRand(2.0f);
        p.mass = 1;

        p.id = i;
        p.nextInList = -1;

        if (p.radius > maxRadius)
        {
            maxRadius = p.radius;
        }
    }

/*  // debug
    spheres[0].position = vec3(-8.4f, -0.4f, -8.4f);
    spheres[0].radius = 0.5f;
    spheres[0].impulse = vec3(0, 0, 0);
    spheres[0].color = vec4(1);
    spheres[0].mass = 1;
    spheres[0].id = 0;
    maxRadius = 0.5f;

    spheres[1].position = vec3(0, 1.5, 0);
    spheres[1].radius = 0.5f;
    spheres[1].impulse = vec3(0, 0, 0);
    spheres[1].mass = 1;
*/
    //upload sphere array to opengl
    sphereBuffer.set(spheres);
    sphereBuffer.setDrawMode(GL_POINTS);

    sphere_interop.registerGLBuffer(sphereBuffer.getVBO());


    reset();
}

void CollisionSystem::reset()
{
    sphere_interop.map();
    void* spheres_ptr = sphere_interop.getDevicePtr();
    CUDA::resetSpheres(static_cast<CUDA::Sphere*>(spheres_ptr), sphereCount, 9, 9, glm::vec3(-8.f, 1.f, -8.f), 2);
    sphere_interop.unmap();
}

void CollisionSystem::update(float dt, CUDA::Plane* planes, int planeCount)
{
    sphere_interop.map();
    void* spheres = sphere_interop.getDevicePtr();

    glm::vec3 colArea(18.f, 60.f, 18.f); // for linked cell. declares the area to partition into cell grid

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


}

void CollisionSystem::render(Camera *cam)
{
    //render the particles from the viewpoint of the camera
    sphereShader->bind();
    sphereShader->uploadAll(mat4(),cam->view,cam->proj);
    sphereBuffer.bindAndDraw();
    sphereShader->unbind();
}

void CollisionSystem::keyPressed(int key)
{
    switch(key)
    {
        case SDLK_r:
        {
            reset();
            break;
        }

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



