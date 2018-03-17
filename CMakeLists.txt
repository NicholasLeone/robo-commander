cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(robo-dev)

# ===========================
#       Set Build Options
# ===========================
set(COMPILE_ARM     ON CACHE INTERNAL "Build targets for ARM processors (e.g Raspberry Pi)")

set(VERBOSE         OFF CACHE INTERNAL "CMake execution debugging")

set(BUILD_ACTUATORS ON CACHE INTERNAL "Build all the hardware drivers used to control various standard actuators")
set(BUILD_COMMS     ON CACHE INTERNAL "Build all the hardware drivers used to handle device communication protocols")
set(BUILD_DEVICES   ON CACHE INTERNAL "Build all the hardware drivers used to")
set(BUILD_SENSORS   ON CACHE INTERNAL "Build all the hardware drivers used to")
set(BUILD_CONTROLS  ON CACHE INTERNAL "Build all the libraries used to handle the control systems")
set(BUILD_FILTERS   ON CACHE INTERNAL "Build all the libraries used to filter and pre-process data")
set(BUILD_UTILS     ON CACHE INTERNAL "Build all the classes useful throughout entire project that help make programming cleaner and quicker")
set(BUILD_PROFILES  ON CACHE INTERNAL "Build all the robot configuration profiles responsible for combining all the usable device drivers to control the hardware")

set(BUILD_TESTS     ON CACHE INTERNAL "Build all test executables for each class")

# ===========================
#        Set Compiler
# ===========================
# if(COMPILE_ARM)
#      set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
#      set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
# endif(COMPILE_ARM)

# ===========================
#    Set CMake Properties
# ===========================
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBS_DIR ${LIBRARY_OUTPUT_PATH})
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(INCS_DIR "${PROJECT_SOURCE_DIR}/include")
set(TEST_DIR "${PROJECT_SOURCE_DIR}/test")

# ============================================
#       Find & Include Dependencies
# ============================================
# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)

# Find Boost Libraries
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads)
find_library(RTIMULib libRTIMULib.so)

# Find pigpiod libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/depends/pigpiod")
find_package(pigpio REQUIRED)

include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${pigpio_INCLUDE_DIR}
     ${LAPACK_INCLUDE_DIRS}
     ${BLAS_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
)

set(LIBS
     ${Boost_LIBRARIES}
     ${pigpiod_if2_LIBRARY}
     ${ARMADILLO_LIBRARIES}
     ${LAPACK_LIBRARIES}
     ${BLAS_LIBRARIES}
     ${CMAKE_THREAD_LIBS_INIT}
)

# ===========================
#     Build Targets
# ===========================
add_subdirectory(src)


# ===========================
#     Install Targets
# ===========================


# ===========================
#         DEBUG
# ===========================
# this is used when searching for include files e.g. using the FIND_PATH() command.
# MESSAGE( STATUS "SHARED: " ${SHARED} )

# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#      message(STATUS "         included='${dir}'")
# endforeach()

# MESSAGE( STATUS "INCLUDE_DIRS: ")
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#      message(STATUS "  dir='${dir}'")
# endforeach()

# this is used when searching for libraries e.g. using the FIND_LIBRARY() command.
# MESSAGE( STATUS "CMAKE_LIBRARY_PATH: " ${CMAKE_LIBRARY_PATH} )

# the compiler flags for compiling C sources
# MESSAGE( STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS} )

# the compiler flags for compiling C++ sources
# MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )
