#include "renderer.h"

#include "saiga/geometry/grid.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include "saiga/opengl/objloader.h"

myRenderer::myRenderer()
{

}

myRenderer::~myRenderer()
{
}

void myRenderer::init()
{
     ShaderLoader::instance()->addPath("shader");
     ObjLoader::instance()->addPath("objs");
     MaterialLoader::instance()->addPath("objs");

    gridShader = ShaderLoader::instance()->load<MVPColorShader>("grid.glsl");

    std::vector<Grid> gs;
    std::vector<CUDA::Plane> ps;

    // bottom
    Grid g(vec3(0),vec3(0,0,1),vec3(1,0,0));
    g.createBuffers(grid_bottom, 10, 10);
    ps.push_back(CUDA::Plane());
    ps[0].center = vec3(0);
    ps[0].normal = normalize(vec3(0, 1, 0));
    ps[0].d     = glm::dot(ps[0].center, ps[0].normal);


    // left
    g = Grid(vec3(-9, 9, 0), vec3(0,0,1), vec3(0,1,0));
    g.createBuffers(grid_left, 10, 10);
    ps.push_back(CUDA::Plane());
    ps[1].center = vec3(-9, 9, 0);
    ps[1].normal = normalize(vec3(1, 0, 0));
    ps[1].d     = glm::dot(ps[1].center, ps[1].normal);

    // right
    g = Grid(vec3(9, 9, 0), vec3(0,0,1), vec3(0,1,0));
    g.createBuffers(grid_right, 10, 10);
    ps.push_back(CUDA::Plane());
    ps[2].center = vec3(9, 9, 0);
    ps[2].normal = normalize(vec3(-1, 0, 0));
    ps[2].d     = glm::dot(ps[2].center, ps[2].normal);

    // front
    g = Grid(vec3(0, 9, 9), vec3(1,0,0), vec3(0,1,0));
    g.createBuffers(grid_front, 10, 10);
    ps.push_back(CUDA::Plane());
    ps[3].center = vec3(0, 9, 9);
    ps[3].normal = normalize(vec3(0, 0, -1));
    ps[3].d     = glm::dot(ps[3].center, ps[3].normal);

    // back
    g = Grid(vec3(0, 9, -9), vec3(1,0,0), vec3(0,1,0));
    g.createBuffers(grid_back, 10, 10);
    ps.push_back(CUDA::Plane());
    ps[4].center = vec3(0, 9, -9);
    ps[4].normal = normalize(vec3(0, 0, 1));
    ps[4].d     = glm::dot(ps[4].center, ps[4].normal);


    planeCount = ps.size();
    planeBuffer.set(ps);
    planeBuffer.setDrawMode(GL_POINTS);

    plane_interop.registerGLBuffer(planeBuffer.getVBO());

    //particleSystem.init();
    collisionSystem.init();

    cout<<"Renderer Initialized!"<<endl;
}

void myRenderer::update(float dt){
    //particleSystem.update(dt);
    plane_interop.map();
    void* plane_ptr = plane_interop.getDevicePtr();
    collisionSystem.update(dt, static_cast<CUDA::Plane*>(plane_ptr), planeCount);
    plane_interop.unmap();
}

void myRenderer::render(Camera *cam)
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    gridShader->bind();
    gridShader->uploadAll(mat4(),cam->view,cam->proj);
    gridShader->uploadColor(vec4(0.7f));
    grid_bottom.bindAndDraw();
    grid_left.bindAndDraw();
    grid_right.bindAndDraw();
    grid_front.bindAndDraw();
    grid_back.bindAndDraw();
    gridShader->unbind();


    //particleSystem.render(cam);
    collisionSystem.render(cam);
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





