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
<<<<<<< HEAD
    float max_lifetime;
=======
	float max_lifetime; // used to integrate colors
>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3

    Particle(){}

}ATTRIBUTES;

    // grid
    void resetParticlesGrid(void* particles, int numberOfParticles, int x, int z, float cornerX, float cornerY, float cornerZ, float distance);
    
    // star
    void integrateParticlesStar(void* particles, int numberOfParticles, float dt);
    
    // volcano
    void integrateParticlesVolcano(void* particles, int numberOfParticles, float maxAngle, float dt);
<<<<<<< HEAD
    void resetParticlesVolcanoAndStar(void* particles, int numberOfParticles);
=======
	void resetParticlesVolcanoAndStar(void* particles, int numberOfParticles);
>>>>>>> cc15008e98bd5bd707b57c1c35f633ac156bf3f3
}
