cmake_minimum_required(VERSION 2.8)

set(PROG_NAME agphys)

set(MAIN_INCLUDE_DIR "" CACHE FILEPATH "Path to yourlib includes")
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} ${MAIN_INCLUDE_DIR}/include)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${MAIN_INCLUDE_DIR}/lib)
include_directories(${MAIN_INCLUDE_DIR}/include)
LINK_DIRECTORIES(${MAIN_INCLUDE_DIR}/lib)   



if(UNIX)
	INCLUDE(FindPkgConfig)
endif(UNIX)

project (${PROG_NAME})
set(CMAKE_BUILD_TYPE Release)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)


set (PROGRAM_VERSION_MAJOR 1)
set (PROGRAM_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")



######### Show all warnings

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic -g")
endif()





#cuda
find_package(CUDA REQUIRED)
find_package(MPI REQUIRED)
INCLUDE(FindCUDA)


############# Required LIBRARIES ###############

#OPENGL
find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}) 
SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})

#GLEW
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS} )
SET(LIBS ${LIBS} ${GLEW_LIBRARIES})


find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${SDL2_LIBRARY})




############# COMPILER FLAGS ###############

if(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

if(WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") #multiprocessor compilation for visual studio
endif(WIN32)
############# SOURCE ###############

add_subdirectory(src)



