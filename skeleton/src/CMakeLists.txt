

#build a static library from all cuda sources

FILE(GLOB_RECURSE CUDA_SRC  *.cu)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_NVCC_FLAGS -arch=sm_20)
CUDA_ADD_LIBRARY(my_cuda_lib ${CUDA_SRC} STATIC)
TARGET_LINK_LIBRARIES(my_cuda_lib ${CUDA_LIBRARIES})





#build executable and link to static cuda library
#FILE(GLOB_RECURSE PROG_SRC  *.cpp)
FILE(GLOB PROG_SRC  *.cpp)
FILE(GLOB PROG_HEADER  *.h)

include_directories(.)
include_directories(saiga)
FILE(GLOB_RECURSE saiga_SRC saiga/*.cpp)
FILE(GLOB_RECURSE saiga_HEADER saiga/*.h)


SET(PROG_SRC ${PROG_SRC} ${saiga_SRC})
SET(PROG_HEADER ${PROG_HEADER}  ${saiga_HEADER} )

add_executable(${PROG_NAME} ${PROG_SRC} ${PROG_HEADER})



if (USE_CUDA)
	target_link_libraries(${PROG_NAME} my_cuda_lib)
endif (USE_CUDA)

target_link_libraries(${PROG_NAME} ${LIBS} my_cuda_lib)



