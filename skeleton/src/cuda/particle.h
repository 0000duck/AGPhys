#pragma once

#include "cuda_typedefs.h"

namespace CUDA{


static const int threadsPerBlock = 128;

class ALIGNMENT Particle
{
public:
    vec3_t position;
    float radius;
    vec4_t color;
    vec3_t impulse;
    float lifetime;
    float max_lifetime;

    Particle(){}

}ATTRIBUTES;

    // grid
    void resetParticlesGrid(void* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance);
    
    // star
    void integrateParticlesStar(void* particles, int numberOfParticles, float dt);
    
    // volcano
    void integrateParticlesVolcano(void* particles, int numberOfParticles, float maxAngle, float dt);
    void resetParticlesVolcanoAndStar(void* particles, int numberOfParticles);
}
