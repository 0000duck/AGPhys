#define KINEMATIC
#define GRAVITY

#include <cstdlib>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include "helper_math.h"	// overload operators for floatN
#include "helper_cuda.h"

// thrust
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/sort.h>


#include "sphere.h"
#include "collision.h"
#include "timing.h"


namespace {
#define checked_cuda(ans) { gpu_assert((ans), __FILE__, __LINE__); }
inline void gpu_assert(cudaError_t code, char *file, int line, bool abort=true) {
    if (code != cudaSuccess) {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}
}

namespace CUDA {

__global__ void resetSpheresGrid(Sphere* spheres, int numberOfSpheres, int x, int z, float cornerX, float cornerY, float cornerZ, float distance)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        int layerSize = x * z;
        int yPos = tid / layerSize;
        int normId = tid - yPos * layerSize;

        int xPos = normId % x;
        int zPos = (normId - xPos) / x;

        spheres[tid].position.x = xPos * distance + cornerX;
        spheres[tid].position.y = yPos * distance + cornerY;
        spheres[tid].position.z = zPos * distance + cornerZ;
    }
}

__global__ void integrateSpheres(Sphere* spheres, int numberOfSpheres, float dt)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& s = spheres[tid];

#ifdef GRAVITY
        s.velocity  += dt * make_float3(0, -1.5, 0);
#endif
        s.position += dt * s.velocity;

    }

}

__global__ void collidePlanes(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        for (int p = 0; p < numberOfPlanes; ++p)
        {
            Plane& plane = planes[p];
            float penetration = collideSpherePlane(sphere, plane);
            if (penetration != -1.0f)
            {
                //sphere.color = make_float4(1);

#ifdef KINEMATIC
                kinematicCollisionResponseSpherePlane(sphere, plane, penetration);
#else
                dynamicCollisionResponseSpherePlane(sphere, plane, penetration);
#endif
            }
        }
    }
}

/// --------------------------------------------------------------------- BRUTE FORCE ---------------------------------------------------------------------

__global__ void collideSpheresBruteForce(Sphere* spheres, int numberOfSpheres)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        Sphere* collider = NULL;
        float max = -1.0f;
        for (int s = 0; s < tid; ++s)
        {
            if (s == tid) continue;
            Sphere& other = spheres[s];
            float penetration = collideSphereSphere(sphere, other);

            if (penetration > max)
            {
                collider = &other;
                max = penetration;
            }
        }

        if (max > -1.0f)
        {

#ifdef KINEMATIC
                kinematicCollisionResponseSphereSphere(sphere, *collider, max);
#else
                elasticCollision(sphere, *collider, max);
#endif

        }

    }
}


/// --------------------------------------------------------------------- SORT & SWEEP --------------------------------------------------------------------

struct ALIGNMENT AxisProjection
{
    float   value;
    bool    startPoint;
    int     sphereID;

} ATTRIBUTES;

struct FillWithStartPoints
{
    __host__ __device__ AxisProjection operator()(const Sphere& s)
    {
        AxisProjection b;
        b.startPoint = true;
        b.value = s.position.y - s.radius;
        b.sphereID = s.id;
        return b;
    }
};

struct FillWithEndPoints
{
    __host__ __device__ AxisProjection operator()(const Sphere& s)
    {
        AxisProjection e;
        e.startPoint = false;
        e.value = s.position.y + s.radius;
        e.sphereID = s.id;
        return e;
    }
};

struct Sort
{
    __host__ __device__ bool operator()(const AxisProjection& a1, const AxisProjection& a2)
    {
        return a1.value < a2.value;
    }
};



__global__ void collideSpheresSortAndSweep(AxisProjection* projections, Sphere* spheres, int numberOfProjections)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfProjections)
    {
        AxisProjection& projection = projections[tid];
        Sphere& sphere = spheres[projection.sphereID];

        if (!projection.startPoint)
        {
            // endpoint
            return;
        }

        Sphere* collider = NULL;
        float max = -1.0f;

        for (int i = tid + 1;; ++i)
        {
            AxisProjection& other_proj = projections[i];
            Sphere& other = spheres[other_proj.sphereID];

            if (sphere.id == other.id)
            {
                // endpoint of own sphere
                break;
            }

            if (!other_proj.startPoint)
            {
                // endpoint of other sphere -> ignore
                continue;
            }

            float penetration = collideSphereSphere(sphere, other);
            if (penetration > max)
            {
                collider = &other;
                max = penetration;
            }
        }

        if (max > -1.0f)
        {

#ifdef KINEMATIC
            kinematicCollisionResponseSphereSphere(sphere, *collider, max);
#else
            elasticCollision(sphere, *collider, max);
#endif

        }
    }
}


/// --------------------------------------------------------------------- LINKED CELL ---------------------------------------------------------------------

__device__ int3 inline computeCellCoords(const float3& position, const float3& cellSize, const float3& corner)
{
    float3 relativeCoords = position - corner;
    return make_int3(floor(relativeCoords.x / cellSize.x), floor(relativeCoords.y / cellSize.y), floor(relativeCoords.z / cellSize.z));
}

/**
  gridSize is the number of cells in each direction, not the total size of the grid
 */
__device__ inline int computeHash(const int3& cellCoords, const int3& gridSize)
{
    return (gridSize.z * cellCoords.x + cellCoords.z) + (cellCoords.y * gridSize.x * gridSize.z);
}

__global__ void collideSpheresLinkedCell(Sphere* spheres, int numberOfSpheres, int* cells, float gridSizeX, float gridSizeY, float gridSizeZ,
                                         float cellSizeX, float cellSizeY, float cellSizeZ, float cornerX, float cornerY, float cornerZ)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        int3 cellCoords = computeCellCoords(sphere.position, make_float3(cellSizeX, cellSizeY, cellSizeZ), make_float3(cornerX, cornerY, cornerZ)); // which cell is sphere in
        int3 gridSize = make_int3(gridSizeX / cellSizeX, gridSizeY / cellSizeY, gridSizeZ / cellSizeZ); // number of cells in each dimension

        float max = -1.0f;
        Sphere* collider = NULL;

        // loop over 3x3 cell neighborhood
        for (int x = -1; x <= 1; ++x)
        {
            for (int y = -1; y <= 1; ++y)
            {
                for (int z = -1; z <= 1; ++z)
                {
                    int3 offset = make_int3(x, y, z);
                    int3 testCell = cellCoords + offset;

                    if (testCell.x < 0 || testCell.y < 0 || testCell.z < 0
                            || testCell.x >= gridSize.x || testCell.y >= gridSize.y || testCell.z >= gridSize.z)
                    {
                        // out of bounds
                        continue;
                    }


                    int hash = computeHash(testCell, gridSize);
                    int cell = cells[hash];

                    while (cell != -1)
                    {
                        Sphere& other = spheres[cell];
                        cell = other.nextInList;
                        if (other.id == sphere.id)
                        {
                            // self
                            continue;
                        }
                        if (other.id > sphere.id)
                        {
                            // as in all methods, I only resolve collisions with spheres with smaller IDs to avoid concurrency problems
                            continue;
                        }

                        float penetration = collideSphereSphere(sphere, other);
                        if (penetration > max)
                        {
                            max = penetration;
                            collider = &other;
                        }
                    }
                }
            }
        }

        if (max > -1.0f)
        {

#ifdef KINEMATIC
                kinematicCollisionResponseSphereSphere(sphere, *collider, max);
#else
                elasticCollision(sphere, *collider, max);
#endif

        }
    }
}

__global__ void initCellGrid(Sphere* spheres, int numberOfSpheres, int* cells, float gridSizeX, float gridSizeY, float gridSizeZ,
                             float cellSizeX, float cellSizeY, float cellSizeZ, float cornerX, float cornerY, float cornerZ)
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < numberOfSpheres)
    {
        Sphere& sphere = spheres[tid];

        int3 cellCoords = computeCellCoords(sphere.position, make_float3(cellSizeX, cellSizeY, cellSizeZ), make_float3(cornerX, cornerY, cornerZ));
        int3 gridSize = make_int3(gridSizeX / cellSizeX, gridSizeY / cellSizeY, gridSizeZ / cellSizeZ); // number of cells in each dimension
        if (cellCoords.x < 0 || cellCoords.y < 0 || cellCoords.z < 0
                || cellCoords.x >= gridSize.x || cellCoords.y >= gridSize.y || cellCoords.z >= gridSize.z)
        {
            // out of bounds
            return;
        }


        int hash = computeHash(cellCoords, make_int3(gridSizeX / cellSizeX, gridSizeY / cellSizeX, gridSizeZ / cellSizeZ));

        int currentEntry = atomicExch(&cells[hash], sphere.id);
        sphere.nextInList = currentEntry;
    }
}






/// ------------------------------------------------------------------------- CPU -------------------------------------------------------------------------

void resetSpheres(Sphere* spheres, int numberOfSpheres, int x, int z, const glm::vec3& corner, float distance)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;
    resetSpheresGrid<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, x, z, corner.x, corner.y, corner.z, distance);
}


static float accTime = 0.0f;
static float accDts = 0.0f;
static int numberOfSamples = 0;


void updateAllSpheresBruteForce(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    startTiming();

    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);
    collidePlanes<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);
    collideSpheresBruteForce<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres);

    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;


    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "BRUTE FORCE: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;
        std::cout.flush();

        accTime = 0.0f;
        accDts  = 0.0f;
        numberOfSamples = 0;
    }
}

void updateAllSpheresSortAndSweep(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    startTiming();

    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);
    collidePlanes<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);

    thrust::device_vector<AxisProjection> projectionVector(numberOfSpheres * 2);

    thrust::device_ptr<Sphere> start(spheres);
    thrust::device_ptr<Sphere> end(spheres + numberOfSpheres);
    thrust::transform(start, end, projectionVector.begin(), FillWithStartPoints());
    thrust::transform(start, end, projectionVector.begin() + numberOfSpheres, FillWithEndPoints());
    thrust::sort(projectionVector.begin(), projectionVector.end(), Sort());

    AxisProjection* projectionPtr = thrust::raw_pointer_cast(projectionVector.data());

    blocks = (numberOfSpheres * 2) / threadsPerBlock + 1;
    collideSpheresSortAndSweep<<<blocks, threadsPerBlock>>>(projectionPtr, spheres, numberOfSpheres * 2);

    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;

    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "SORT & SWEEP: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;

        accTime = 0.0f;
        accDts = 0.0f;
        numberOfSamples = 0;
    }

}

void updateAllSpheresLinkedCell(Sphere* spheres, Plane* planes, int numberOfSpheres, int numberOfPlanes, float dt, const glm::vec3 &dim_colDomain, const glm::vec3 &offset_colDomain, float maxRadius)
{
    int threadsPerBlock = 128;
    int blocks = numberOfSpheres / threadsPerBlock + 1;

    startTiming();

    integrateSpheres<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, dt);
    collidePlanes<<<blocks, threadsPerBlock>>>(spheres, planes, numberOfSpheres, numberOfPlanes);

    // create 3d grid
    glm::vec3 numberOfCells = dim_colDomain / (maxRadius * 2);
    glm::floor(numberOfCells);
    glm::vec3 cellSize(dim_colDomain.x / numberOfCells.x, dim_colDomain.y / numberOfCells.y, dim_colDomain.z / numberOfCells.z);
    thrust::device_vector<int> cells(numberOfCells.x * numberOfCells.y * numberOfCells.z, -1);

    int* cellPtr = thrust::raw_pointer_cast(cells.data());
    initCellGrid<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, cellPtr, dim_colDomain.x, dim_colDomain.y, dim_colDomain.z,
                                              cellSize.x, cellSize.y, cellSize.z, offset_colDomain.x, offset_colDomain.y, offset_colDomain.z);


    collideSpheresLinkedCell<<<blocks, threadsPerBlock>>>(spheres, numberOfSpheres, cellPtr, dim_colDomain.x, dim_colDomain.y, dim_colDomain.z,
                                                          cellSize.x, cellSize.y, cellSize.z, offset_colDomain.x, offset_colDomain.y, offset_colDomain.z);

    float time = endTiming();

    numberOfSamples++;
    accTime += time;
    accDts  += dt;

    if (accDts >= 1.0f)
    {
        float timePerFrame = accTime / numberOfSamples;
        std::cout << "LINKED CELL: Number of Spheres: " << numberOfSpheres << ", Average Step Time: " << timePerFrame << "ms" << std::endl;

        accTime = 0.0f;
        accDts = 0.0f;
        numberOfSamples = 0;
    }

    checked_cuda(cudaDeviceSynchronize()); // necessary?
}







}
