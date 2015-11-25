#include "collisionSystem.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include <random>
#include <ctime>


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
    for(CUDA::Sphere& p : spheres){
        p.position = glm::ballRand(5.0f) + vec3(0, 5, 0);
        p.radius = 0.1f * (rand() % 2 + 3);
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
        p.impulse = glm::ballRand(5.0f);
        p.mass = (rand() % 8) + 2;
    }
    /*
    spheres[0].position = vec3(-5, 10, 0);
    spheres[0].radius   = 0.5f;
    spheres[0].color    = vec4(1);
    spheres[0].impulse  = vec3(1, 0, 0);
    spheres[0].mass     = 10;

    spheres[1].position = vec3(5, 10, 0);
    spheres[1].radius   = 0.5f;
    spheres[1].color    = vec4(1);
    spheres[1].impulse  = vec3(-1, 0, 0);
    spheres[1].mass     = 10;
    */

    //upload sphere array to opengl
    sphereBuffer.set(spheres);
    sphereBuffer.setDrawMode(GL_POINTS);

    sphere_interop.registerGLBuffer(sphereBuffer.getVBO());

    sphere_interop.map();
    void* spheres_ptr = sphere_interop.getDevicePtr();
    CUDA::resetSpheres(static_cast<CUDA::Sphere*>(spheres_ptr), sphereCount, 9, 9, -8, 1, -8, 2);
    sphere_interop.unmap();
}

void CollisionSystem::update(float dt, CUDA::Plane* planes, int planeCount)
{
    sphere_interop.map();
    void* spheres = sphere_interop.getDevicePtr();

    // TODO: update
    CUDA::updateAllSpheres(static_cast<CUDA::Sphere*>(spheres), static_cast<CUDA::Plane*>(planes), sphereCount, planeCount, dt);

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
    switch(key){
        case SDLK_r:
            sphere_interop.map();
            void* spheres_ptr = sphere_interop.getDevicePtr();
            CUDA::resetSpheres(static_cast<CUDA::Sphere*>(spheres_ptr), sphereCount, 10, 10, -7, 0, -7, 1);
            sphere_interop.unmap();
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



