#pragma once

#include "saiga/util/glm.h"
#include "sdl/sdl_eventhandler.h"

#include <array>


template<typename camera_t>
class Controllable_Camera : public SDL_KeyListener, public SDL_MouseListener{
public:
    bool dragging = false;


    camera_t* cam;
    float movementSpeed = 3;

    float rotationSpeed = 0.2f;

    enum Key{
        Forward = 0,
        Backward = 1,
        Left = 2,
        Right =3,
        Fast =4
    };
    std::array<int,5> sdlKeys ;

    std::array<bool,5> keyStates {};

    int buttonDrag = SDL_BUTTON_LEFT;

    Controllable_Camera(camera_t* cam):cam(cam){
        cam->rot = glm::quat_cast(cam->model);
        sdlKeys = {SDLK_w,SDLK_s,SDLK_a,SDLK_d,SDLK_0};
    }

    ~Controllable_Camera(){ }

    void update(float delta);
    void setPosition(const glm::vec3 &cords);


    void keyPressed(int key);
    void keyReleased(int key);

    void mouseMoved(int relx, int rely);
    void mousePressed(int key, int x, int y) ;
    void mouseReleased(int key, int x, int y) ;
};

template<class camera_t>
void Controllable_Camera<camera_t>::keyPressed(int key)
{
    for(int i = 0 ; i < 5 ; ++i){
        if(key==sdlKeys[i]){
            keyStates[i] = true;
        }
    }

}

template<class camera_t>
void Controllable_Camera<camera_t>::keyReleased(int key)
{
    for(int i = 0 ; i < 5 ; ++i){
        if(key==sdlKeys[i]){
            keyStates[i] = false;
        }
    }

}

template<class camera_t>
void Controllable_Camera<camera_t>::mouseMoved(int relx, int rely)
{
    if(dragging){
        cam->turn((float)-relx*rotationSpeed,(float)-rely*rotationSpeed);
        cam->calculateModel();
        cam->updateFromModel();
    }
}

template<class camera_t>
void Controllable_Camera<camera_t>::mousePressed(int key, int x, int y)
{
    if(key==buttonDrag){
        dragging = true;
    }
}

template<class camera_t>
void Controllable_Camera<camera_t>::mouseReleased(int key, int x, int y)
{
    if(key==buttonDrag){
        dragging = false;
    }
}


template<class camera_t>
void Controllable_Camera<camera_t>::setPosition(const glm::vec3& cords)
{
    cam->setPosition(cords);
    cam->calculateModel();
}

template<class camera_t>
void Controllable_Camera<camera_t>::update(float delta){
    int FORWARD =  keyStates[Forward] - keyStates[Backward];
     int RIGHT = keyStates[Right] - keyStates[Left];


    vec3 trans = delta*movementSpeed*FORWARD*vec3(0,0,-1) + delta*movementSpeed*RIGHT*vec3(1,0,0);
    cam->translateLocal(trans);
    cam->calculateModel();
    cam->updateFromModel();
}

