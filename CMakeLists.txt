cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(robo-commander)

# ===========================
#       Set Build Options
# ===========================
set(COMPILE_ARM     OFF CACHE INTERNAL "Build targets for ARM processors (e.g Raspberry Pi)")

set(VERBOSE         OFF CACHE INTERNAL "CMake execution debugging")

set(BUILD_ACTUATORS ON CACHE INTERNAL "Build all the hardware drivers used to control various standard actuators")
set(BUILD_COMMS     ON CACHE INTERNAL "Build all the hardware drivers used to handle device communication protocols")
set(BUILD_DEVICES   ON CACHE INTERNAL "Build all the hardware drivers used to")
set(BUILD_SENSORS   ON CACHE INTERNAL "Build all the hardware drivers used to")
set(BUILD_CONTROLS  ON CACHE INTERNAL "Build all the libraries used to handle the control systems")
set(BUILD_FILTERS   ON CACHE INTERNAL "Build all the libraries used to filter and pre-process data")
set(BUILD_UTILS     ON CACHE INTERNAL "Build all the classes useful throughout entire project that help make programming cleaner and quicker")
set(BUILD_PROFILES  ON CACHE INTERNAL "Build all the robot configuration profiles responsible for combining all the usable device drivers to control the hardware")
set(BUILD_ROBOTS    ON CACHE INTERNAL "Build all the robot configuration profiles responsible for combining all the usable device drivers to control the hardware")

set(WITH_GTSAM      OFF CACHE INTERNAL "Include the GTSAM library")
set(WITH_OPENCV     OFF CACHE INTERNAL "Include the OpenCV library")

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
set(LIBRARY_OUTPUT_PATH "lib")
set(EXECUTABLE_OUTPUT_PATH "bin")
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBS_DIR ${LIBRARY_OUTPUT_PATH})
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(INCS_DIR "${PROJECT_SOURCE_DIR}/include")
set(TEST_DIR "${PROJECT_SOURCE_DIR}/tests")
set(HOME_DIR "${PROJECT_SOURCE_DIR}")

# ============================================
#       Find & Include Dependencies
# ============================================
# Find Boost Libraries
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads)
# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)
set(EXTERNAL_PACKAGE_INCLUDES "${LAPACK_INCLUDE_DIRS}" "${BLAS_INCLUDE_DIRS}" "${ARMADILLO_INCLUDE_DIRS}")
set(EXTERNAL_PACKAGE_LIBRARIES "${LAPACK_LIBRARIES}" "${BLAS_LIBRARIES}" "${ARMADILLO_LIBRARIES}")

# Find library for IMU
find_library(RTIMULib libRTIMULib.so)

# Find pigpiod libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/external/pigpiod")
find_package(pigpio REQUIRED)

# Find GTSAM libraries
if(WITH_GTSAM)
     find_package(GTSAMCMakeTools)
     find_package(GTSAM REQUIRED)
     list(APPEND EXTERNAL_PACKAGE_INCLUDES "${GTSAM_INCLUDE_DIR}")
     list(APPEND EXTERNAL_PACKAGE_LIBRARIES gtsam)
endif(WITH_GTSAM)

# Find OpenCV libraries
if(WITH_OPENCV)
     find_package(OpenCV REQUIRED)
     list(APPEND EXTERNAL_PACKAGE_INCLUDES "${OpenCV_INCLUDE_DIRS}")
     list(APPEND EXTERNAL_PACKAGE_LIBRARIES "${OpenCV_LIBS}")
endif(WITH_OPENCV)

# For PurePursuit Controller
if(BUILD_CONTROLS)
     find_package(PythonLibs 2.7)
     include_directories(
     	${PYTHON_INCLUDE_DIRS}
     	/usr/local/lib/python2.7/dist-packages/numpy/core/include
     )
endif(BUILD_CONTROLS)

include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${pigpio_INCLUDE_DIR}
     ${EXTERNAL_PACKAGE_INCLUDES}
)

set( RoboCommander_LIBRARIES
     ${Boost_LIBRARIES}
     ${pigpiod_if2_LIBRARY}
     ${CMAKE_THREAD_LIBS_INIT}
)

set( RoboCommander_INCLUDE_DIRS
     "${INCS_DIR}/actuators"
     "${INCS_DIR}/algorithms"
     "${HOME_DIR}/external/TinyEKF/src"
     "${INCS_DIR}/communication"
     "${INCS_DIR}/controllers"
     "${INCS_DIR}/devices"
     "${INCS_DIR}/robots"
     "${INCS_DIR}/sensors"
     "${INCS_DIR}/utilities"
     "${INCS_DIR}/vehicle_profiles"
)

# ===========================
#     Build Targets
# ===========================
add_subdirectory(src)

if(BUILD_TESTS)
     add_subdirectory(tests)
endif(BUILD_TESTS)

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
