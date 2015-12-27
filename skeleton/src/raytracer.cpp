#include "raytracer.h"
#include <glm/glm.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>

struct Ray
{
    glm::vec3 origin;
    glm::vec3 dir;
};

struct Intersection
{
    glm::vec3 pos;
    float t;
};

static bool sortIntersections(const Intersection& i1, const Intersection& i2)
{
    return i1.t < i2.t;
}

static bool triangleIntersection(Ray& ray, Triangle& tri, float maxT, Intersection& intersect)
{
    glm::vec3 x0 = ray.origin;
    glm::vec3 x1 = ray.origin + maxT * ray.dir;
    glm::vec3 D = (x1 - x0);

    if (D.length() < 0.00001)
        return false;

    glm::vec3 e1 = tri.b - tri.a;
    glm::vec3 e2 = tri.c - tri.a;
    glm::vec3 P = glm::cross(D, e2);
    float det = glm::dot(e1, P);

    if (det > -0.00001f && det < 0.00001f)
        return false;

    float invDet = 1.0f / det;
    glm::vec3 T = x0 - tri.a;
    float u = glm::dot(T, P) * invDet;

    if (u < 0.0f || u > 1.0f)
        return false;

    glm::vec3 Q = glm::cross(T, e1);
    float v = glm::dot(D, Q) * invDet;

    if (v < 0.0f || u + v > 1.0f)
        return false;

    // along the ray
    float t = glm::dot(e2, Q) * invDet;

    float t_min = 0.00001f;
    float t_max = 1.0f + 0.00001f;

    if (t < t_min || t > t_max)
        return false;

    float weightX = 1.0f - u - v;
    float weightY = u;
    float weightZ = v;
    intersect.pos = weightX * tri.a + weightY * tri.b + weightZ * tri.c;

    if (t >= t_min && t <= t_max) {
        // segment intersects triangle
        intersect.t = t;
        return true;
    }

    if (abs(t) < 0.00001) {
        // x0 case
        intersect.t = 0.0;
        return true;
    }

    if (abs(t_max - t) < 0.00001) {
        // x1 case
        intersect.t = 1.0;
        return true;
    }

    // undefined case
    return false;
}

void RayTracer::fillBufferWithSpheres(MaterialMesh<VertexNT,GLuint>* input, std::vector<CUDA::Sphere>& output, float sphereRadius, float mass)
{
    aabb box = input->calculateAabb();
    glm::vec3 max = box.max;
    glm::vec3 min = box.min;

    std::vector<VertexNT>& vertices = input->vertices;
    std::vector<TriangleMesh<VertexNT,GLuint>::Face>& faces = input->faces;
    float max_t = max.z - min.z;

    std::cout << box << std::endl;

    for (float x = min.x + sphereRadius; x < max.x; x += sphereRadius * 2.f)
    {
        for (float y = min.y + sphereRadius; y < max.y; y += sphereRadius * 2.f)
        {
            //std::cout << x << " " << y << std::endl;
            Ray ray;
            ray.origin  = glm::vec3(x, y, min.z);
            ray.dir     = glm::vec3(0.f, 0.f, 1);

            std::vector<Intersection> intersects;

            for (auto face : faces)
            {
                Triangle tri;
                tri.a = vertices[face.v1].position;
                tri.b = vertices[face.v2].position;
                tri.c = vertices[face.v3].position;

                Intersection intersect;
                if (triangleIntersection(ray, tri, max_t, intersect))
                {
                    intersects.push_back(intersect);
                }
            }

            std::sort(intersects.begin(), intersects.end(), sortIntersections);
            //std::cout << intersects.size() << std::endl;
            assert(intersects.size() % 2 == 0);

            if (intersects.size() > 0)
            {
                for (unsigned int i = 0; i < intersects.size(); i += 2)
                {
                    Intersection& begin = intersects[i];
                    Intersection& end = intersects[i + 1];

                    for (float s = begin.pos.z + sphereRadius; s < end.pos.z; s += sphereRadius * 2.f)
                    {
                        //std::cout << "create sphere" << std::endl;
                        CUDA::Sphere sphere;
                        sphere.position = glm::vec3(x, y, s);
                        sphere.radius = sphereRadius;
                        sphere.color = glm::vec4(1.f, 0.f, 0.f, 1.f);
                        sphere.velocity = glm::vec3(0.f);
                        sphere.id = -1;
                        output.push_back(sphere);
                    }
                }

            }
        }
    }

    float massPerSphere = mass / (float) output.size();

    for (CUDA::Sphere& s : output)
    {
        s.mass = massPerSphere;
    }
}

void RayTracer::fillBufferWithSpheres(std::vector<CUDA::Sphere> &output, float sphereRadius, float mass)
{
    for (float y = -sphereRadius - sphereRadius; y <= sphereRadius + sphereRadius; y += sphereRadius * 2.f)
    {
        for (float z = -sphereRadius - sphereRadius; z <= sphereRadius + sphereRadius; z += sphereRadius * 2.f)
        {
            for (float x = -sphereRadius - sphereRadius; x <= sphereRadius + sphereRadius; x += sphereRadius * 2.f)
            {
                CUDA::Sphere sphere;
                sphere.position = glm::vec3(x, y, z);
                sphere.radius = sphereRadius;
                sphere.color = glm::vec4(1.f, 0.f, 0.f, 1.f);
                sphere.velocity = glm::vec3(0.f);
                sphere.mass = mass / 27.f;
                sphere.id = -1;
                output.push_back(sphere);
            }
        }
    }
}
