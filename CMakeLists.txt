cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(RoboCommander)

set(RoboCommander_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(RoboCommander_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(RoboCommander_VERSION ${RoboCommander_VERSION_MAJOR}.${RoboCommander_VERSION_MINOR} CACHE STRING "version" FORCE)

# ===========================
#       Set Build Options
# ===========================
set(VERBOSE         OFF CACHE INTERNAL "CMake execution debugging")
set(BUILD_TESTS     ON CACHE INTERNAL "Build all test executables for each class")

set(WITH_GTSAM      OFF CACHE INTERNAL "Include the GTSAM library")
set(WITH_OPENCV     OFF CACHE INTERNAL "Include the OpenCV library")

set(COMPILE_ARM     OFF CACHE INTERNAL "Build targets for ARM processors (e.g Raspberry Pi)")

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
set(LIBS_DIR ${LIBRARY_OUTPUT_PATH})
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(INCS_DIR "${PROJECT_SOURCE_DIR}/include")
set(TEST_DIR "${PROJECT_SOURCE_DIR}/tests")
set(HOME_DIR "${PROJECT_SOURCE_DIR}")

set(EXEC_DEST_DIR "bin/${PROJECT_NAME}-${RoboCommander_VERSION}")
set(INC_DEST_DIR "include/${PROJECT_NAME}-${RoboCommander_VERSION}")
set(PRJOECT_LIB_DEST "lib/${PROJECT_NAME}-${RoboCommander_VERSION}")

# set(CMAKE_INSTALL_PREFIX "outputs")

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
find_package(PythonLibs 2.7)

include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${pigpio_INCLUDE_DIR}
     ${EXTERNAL_PACKAGE_INCLUDES}
     # For PurePursuit Controller
     ${PYTHON_INCLUDE_DIRS}
     /usr/local/lib/python2.7/dist-packages/numpy/core/include
)

set( RoboCommander_EXTERNAL_LIBRARIES
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

file(GLOB RoboCommander_HEADERS
     "${INCS_DIR}/actuators/*.h"
     "${INCS_DIR}/algorithms/*.h"
     "${INCS_DIR}/communication/*.h"
     "${INCS_DIR}/controllers/*.h"
     "${INCS_DIR}/devices/*.h"
     "${INCS_DIR}/robots/*.h"
     "${INCS_DIR}/sensors/*.h"
     "${INCS_DIR}/utilities/*.h"
     "${INCS_DIR}/vehicle_profiles/*.h"
     "${INCS_DIR}/base/*.h"
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
# include(GNUInstallDirs)

install(
     EXPORT RoboCommander_TARGETS
     DESTINATION "${PRJOECT_LIB_DEST}/cmake/RoboCommander"
     NAMESPACE RoboCommander::
     FILE RoboCommanderPkg.cmake # Not sure if this is still needed
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
     "RoboCommanderConfig.cmake.in"
     "RoboCommanderConfig.cmake"
     INSTALL_DESTINATION "${PRJOECT_LIB_DEST}/cmake/RoboCommander"
     # PATH_VARS ${PRJOECT_LIB_DEST}
)

write_basic_package_version_file(
     ${CMAKE_CURRENT_BINARY_DIR}/RoboCommanderConfigVersion.cmake
     VERSION ${RoboCommander_VERSION}
     COMPATIBILITY AnyNewerVersion
)

install(
     FILES
          "${CMAKE_CURRENT_BINARY_DIR}/RoboCommanderConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/RoboCommanderConfigVersion.cmake"
     DESTINATION
          "${PRJOECT_LIB_DEST}/cmake/RoboCommander"
)

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
