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

    //gridShader = ShaderLoader::instance()->load<MVPColorShader>("grid.glsl");
    //Grid g(vec3(0),vec3(0,0,1),vec3(1,0,0));
    //g.createBuffers(grid,10,10);

    //particleSystem.init();
    collisionSystem.init();

    cout<<"Renderer Initialized!"<<endl;
}

void myRenderer::update(float dt){
    //particleSystem.update(dt);
    collisionSystem.update(dt);
}

void myRenderer::render(Camera *cam)
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    //gridShader->bind();
    //gridShader->uploadAll(mat4(),cam->view,cam->proj);
    //gridShader->uploadColor(vec4(0.7f));
    //grid.bindAndDraw();
    //gridShader->unbind();


    //particleSystem.render(cam);
    collisionSystem.render(cam);
}







