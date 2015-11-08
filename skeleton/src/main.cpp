
#include <iostream>

#include "sdl_window.h"

//because sdl has a stupid define
#undef main



int main( int argc, char* args[] )
{

    sdl_Window* window = new sdl_Window("Agphys",1600,900);



    window->startMainLoop();


    std::cout<<"Quitting..."<<std::endl;

    delete window;
}
