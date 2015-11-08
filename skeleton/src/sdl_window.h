#pragma once

#include <SDL2/SDL.h>

#include "controllable_camera.h"
#include "camera.h"

#include "renderer.h"


class sdl_Window : public SDL_KeyListener{
protected:

    std::string name;
    int width;
    int height;
    bool running = false;
    PerspectiveCamera cam;
    Controllable_Camera<PerspectiveCamera> ccam;


    SDL_Window* gWindow = NULL;
    SDL_GLContext gContext;

    myRenderer renderer;

    bool initWindow();
    bool initInput();
public:
    SDL_EventHandler eventHandler;

    sdl_Window(const std::string &name,int width,int height);



    void close();
    void startMainLoop();

    void update(float f){}

    void keyPressed(int key);
    void keyReleased(int key);

};

