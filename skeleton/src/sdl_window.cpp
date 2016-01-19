
#include "sdl_window.h"

sdl_Window::sdl_Window(const std::string &name, int width, int height):name(name),width(width),height(height),cam("test cam"),ccam(&cam)
{
    float aspect = (float)width/(float)height;
    cam.setProj(60.0f,aspect,0.1f,1000.0f);
    cam.setView(vec3(0,0,-10),vec3(0),vec3(0,1,0));

    ccam.setPosition(vec3(0,2,100));

    initWindow();

    eventHandler.addKeyListener(&ccam);
    eventHandler.addMouseListener(&ccam);

    eventHandler.addKeyListener(this);

    eventHandler.addKeyListener(&renderer.particleSystem);
    eventHandler.addKeyListener(&renderer.collisionSystem);
}

bool sdl_Window::initWindow()
{
    //Initialization flag
    bool success = true;

    //Initialize SDL
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 ){
        printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
        return false;
    }

    //Use OpenGL 3.1 core
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 3 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );

    //enable opengl debugging
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);

    //stencil buffer
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    //Create window
    gWindow = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN );
    if( gWindow == NULL ){
        printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
        return false;
    }

    //Create context
    gContext = SDL_GL_CreateContext( gWindow );
    if( gContext == NULL ){
        printf( "OpenGL context could not be created! SDL Error: %s\n", SDL_GetError() );
        return false;
    }

//    glbinding::Binding::initialize();
//    initOpenGL();
    //Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum glewError = glewInit();
    if( glewError != GLEW_OK ){
        printf( "Error initializing GLEW! %s\n", glewGetErrorString( glewError ) );
    }

    //Use Vsync
    if( SDL_GL_SetSwapInterval( 1 ) < 0 ){
        printf( "Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError() );
    }





    cout<<"Opengl version: "<<glGetString(  GL_VERSION)<<endl;


    return success;
}

bool sdl_Window::initInput(){
    //Enable text input
    SDL_StartTextInput();
    return true;
}


void sdl_Window::close()
{

    //Disable text input
    SDL_StopTextInput();

    //Destroy window
    SDL_DestroyWindow( gWindow );
    gWindow = NULL;

    //Quit SDL subsystems
    SDL_Quit();
}

void sdl_Window::startMainLoop(){

    float dt = 1.0/60.0;
    renderer.init();

    running = true;

    while( running ){
        eventHandler.update();
        running &= !eventHandler.shouldQuit();


        ccam.update(dt);
        if (!paused)
            renderer.update(dt);
        renderer.render(&cam);
        SDL_GL_SwapWindow( gWindow );
    }

    renderer.shutdown();

}

void sdl_Window::keyPressed(int key)
{
    switch(key){
        case SDLK_ESCAPE:
            running = false;
            break;
        case SDLK_p:
            paused = !paused;
            break;
    }
}

void sdl_Window::keyReleased(int key)
{

}





