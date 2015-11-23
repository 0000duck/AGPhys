#include "collisionSystem.h"
#include "saiga/opengl/shader/shaderLoader.h"


CollisionSystem::CollisionSystem()
{

}

CollisionSystem::~CollisionSystem()
{
}

void CollisionSystem::init()
{

    sphereShader = ShaderLoader::instance()->load<MVPShader>("particles.glsl");

    //initialize spheres with some random values
    std::vector<CUDA::Sphere> spheres(sphereCount);
    for(CUDA::Sphere& p : spheres){
        p.position = glm::ballRand(5.0f) + vec3(0, 5, 0);
        p.radius = 0.5f;
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
        p.impulse = vec3(20, -1, 0);
    }


    //upload sphere array to opengl
    sphereBuffer.set(spheres);
    sphereBuffer.setDrawMode(GL_POINTS);

    sphere_interop.registerGLBuffer(sphereBuffer.getVBO());
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



