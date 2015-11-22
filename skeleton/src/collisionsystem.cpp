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
    planeShader = ShaderLoader::instance()->load<MVPShader>("plane.glsl");


    //initialize spheres with some random values
    std::vector<CUDA::Sphere> spheres(sphereCount);
    for(CUDA::Sphere& p : spheres){
        p.position = glm::ballRand(5.0f);
        p.radius = 0.5f;
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
       // p.impulse = vec3(0, -1, 0);
    }


/*
    // TEST
    std::vector<CUDA::Sphere> spheres(2);
    spheres[0].position = vec3(0, 10, 0);
    spheres[0].radius = 0.5f;
    spheres[0].impulse = vec3(0, -1, 0);
    spheres[0].color = vec4(1);

    spheres[1].position = vec3(0, 5, 0);
    spheres[1].radius = 0.5f;
    spheres[1].impulse = vec3(0, -1, 0);
    spheres[1].color = vec4(1);
*/

    //upload sphere array to opengl
    sphereBuffer.set(spheres);
    sphereBuffer.setDrawMode(GL_POINTS);

    sphere_interop.registerGLBuffer(sphereBuffer.getVBO());


    // initialize planes
    planeCount = 2;
    std::vector<CUDA::Plane> planes(planeCount);
    planes[0].center = vec3(10, 0, 0);
    planes[0].normal = normalize(vec3(-1, 1, 0));
    planes[0].d     = glm::dot(planes[0].center, planes[0].normal);
    planes[0].d1    = vec3(10, 10, 0);
    planes[0].d2    = vec3(0, 0, 10);

    planes[1].center = vec3(-10, 0, 0);
    planes[1].normal = normalize(vec3(1, 1, 0));
    planes[1].d     = glm::dot(planes[0].center, planes[0].normal);
    planes[1].d1    = vec3(-10, 10, 0);
    planes[1].d2    = vec3(0, 0, 10);

    planeBuffer.set(planes);
    planeBuffer.setDrawMode(GL_POINTS);

    plane_interop.registerGLBuffer(planeBuffer.getVBO());


    //interop.map();
    //void* devPtr = interop.getDevicePtr();

    // TODO: init

    //interop.unmap();
}

void CollisionSystem::update(float dt){
    sphere_interop.map();
    void* spheres = sphere_interop.getDevicePtr();
    plane_interop.map();
    void* planes = plane_interop.getDevicePtr();

    // TODO: update
    CUDA::updateAllSpheres(static_cast<CUDA::Sphere*>(spheres), static_cast<CUDA::Plane*>(planes), sphereCount, planeCount, dt);

    plane_interop.unmap();
    sphere_interop.unmap();
}

void CollisionSystem::render(Camera *cam)
{
    //render the particles from the viewpoint of the camera
    sphereShader->bind();
    sphereShader->uploadAll(mat4(),cam->view,cam->proj);
    sphereBuffer.bindAndDraw();
    sphereShader->unbind();

    planeShader->bind();
    planeShader->uploadAll(mat4(), cam->view, cam->proj);
    planeBuffer.bindAndDraw();
    planeShader->unbind();
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

template<>
void VertexBuffer<CUDA::Plane>::setVertexAttributes()
{
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glEnableVertexAttribArray( 2 );
    glEnableVertexAttribArray( 3 );

    //center
    glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Plane), NULL );
    //normal
    glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Plane), (void*) (3 * sizeof(GLfloat)) );
    //d1
    glVertexAttribPointer(2,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Plane), (void*) (6 * sizeof(GLfloat)) );
    //d2
    glVertexAttribPointer(3,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Plane), (void*) (9 * sizeof(GLfloat)) );

}



