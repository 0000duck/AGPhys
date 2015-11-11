#include "particleSystem.h"

#include "saiga/geometry/grid.h"
#include "saiga/opengl/shader/shaderLoader.h"

#define GRID
//#define STAR
//#define VOLCANO


ParticleSystem::ParticleSystem()
{

}

ParticleSystem::~ParticleSystem()
{
}

void ParticleSystem::init()
{

    particleShader = ShaderLoader::instance()->load<MVPShader>("particles.glsl");

    //initialize particles with some random values
    std::vector<CUDA::Particle> particles(particleCount);
    for(CUDA::Particle& p : particles){
        p.position = glm::ballRand(5.0f);
        p.radius = 0.5f;
        p.color = glm::linearRand(vec4(0,0,0,1),vec4(1,1,1,1));
    }
    //upload particle array to opengl
    particleBuffer.set(particles);
    particleBuffer.setDrawMode(GL_POINTS);

    //run the first test kernel to check if cuda works
    CUDA::test();
    
    interop.registerGLBuffer(particleBuffer.getVBO());
    
    interop.map();
    void* devPtr = interop.getDevicePtr();
    
    // to change the system's type, modify the defines at the top of this file
     
    #ifdef GRID
    int xGrid = 5;
    int zGrid = 9;
    float cornerX = 1.0f, cornerY = 0.0f, cornerZ = 0.0f;
    float distance = 2.0f;
    CUDA::resetParticlesGrid(devPtr, particleCount, xGrid, zGrid, cornerX, cornerY, cornerZ, distance);
    #endif
    
    #ifdef STAR
    CUDA::resetParticlesVolcanoAndStar(devPtr, particleCount);
    #endif
    
    #ifdef VOLCANO
    CUDA::resetParticlesVolcanoAndStar(devPtr, particleCount);
    #endif
    
    interop.unmap();
}

void ParticleSystem::update(float dt){
    //Modify particle buffer with cuda here
    interop.map();
    void* devPtr = interop.getDevicePtr();
    #ifdef GRID
    // do nothing
    #endif
    
    #ifdef STAR
    CUDA::integrateParticlesStar(devPtr, particleCount, dt);
    #endif
    
    #ifdef VOLCANO
    float maxAngle = 30.f; // Angle of volcano outburst in degrees
    CUDA::integrateParticlesVolcano(devPtr, particleCount, maxAngle, dt);
    #endif
    interop.unmap();
}

void ParticleSystem::render(Camera *cam)
{
    //render the particles from the viewpoint of the camera
    particleShader->bind();
    particleShader->uploadAll(mat4(),cam->view,cam->proj);
    particleBuffer.bindAndDraw();
    particleShader->unbind();
}

void ParticleSystem::keyPressed(int key)
{
    switch(key){
        case SDLK_a:
            cout<<"A pressed :)"<<endl;
            break;
    }
}

void ParticleSystem::keyReleased(int key)
{

}


template<>
void VertexBuffer<CUDA::Particle>::setVertexAttributes(){
    //setting the vertex attributes correctly is required, so that the particle shader knows how to read the input data.
    //adding or removing members from the particle class may or may not requires you to change the vertexAttribPointers.

    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glEnableVertexAttribArray( 2 );

    //position
    glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, sizeof(CUDA::Particle), NULL );
    //radius
    glVertexAttribPointer(1,1, GL_FLOAT, GL_FALSE, sizeof(CUDA::Particle), (void*) (3 * sizeof(GLfloat)) );
    //color
    glVertexAttribPointer(2,4, GL_FLOAT, GL_FALSE, sizeof(CUDA::Particle), (void*) (4 * sizeof(GLfloat)) );

}




