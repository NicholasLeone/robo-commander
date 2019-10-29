cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(RoboCommander)

set(RoboCommander_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(RoboCommander_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(RoboCommander_VERSION ${RoboCommander_VERSION_MAJOR}.${RoboCommander_VERSION_MINOR} CACHE STRING "version" FORCE)

# ===========================
#       Set Build Options
# ===========================
set(VERBOSE              OFF CACHE INTERNAL "CMake execution debugging")
set(BUILD_TESTS          ON CACHE INTERNAL "Build all test executables for each class")

set(WITH_GTSAM           OFF CACHE INTERNAL "Include the GTSAM library")
set(WITH_OPENCV          ON CACHE INTERNAL "Include the OpenCV library")
set(WITH_LIBREALSENSE    ON CACHE INTERNAL "Include the librealsense2 library")
set(WITH_OPENMP          ON CACHE INTERNAL "Include the OpenMP library")

set(COMPILE_ARM          OFF CACHE INTERNAL "Build targets for ARM processors (e.g Raspberry Pi)")

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

set(EXEC_DEST_DIR "bin/${PROJECT_NAME}")
set(INC_DEST_DIR "include/${PROJECT_NAME}")
set(PRJOECT_LIB_DEST "lib/${PROJECT_NAME}")
set(PRJOECT_CMAKE_DEST "lib/cmake/${PROJECT_NAME}")

set(INSTALL_LIB_DIR lib) # CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin) # CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include) # CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR lib/cmake) # PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
     set(var INSTALL_${p}_DIR)
     if(NOT IS_ABSOLUTE "${${var}}")
          set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
     endif()
endforeach()

# set(CMAKE_INSTALL_PREFIX "outputs")

# ============================================
#       Find & Include Dependencies
# ============================================
# Find Boost Libraries
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads)
find_package(yaml-cpp REQUIRED)
# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)
set(EXTERNAL_PACKAGE_INCLUDES "${LAPACK_INCLUDE_DIRS}" "${BLAS_INCLUDE_DIRS}" "${ARMADILLO_INCLUDE_DIRS}" "${${YAML_CPP_INCLUDE_DIR}}")
set(EXTERNAL_PACKAGE_LIBRARIES "${LAPACK_LIBRARIES}" "${BLAS_LIBRARIES}" "${ARMADILLO_LIBRARIES}" "${YAML_CPP_LIBRARIES}")

# Find library for IMU
find_library(RTIMULib libRTIMULib.so)

# Find pigpiod libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/external/pigpiod")
find_package(pigpio REQUIRED)

# Find GTSAM libraries
if(WITH_OPENMP)

     # find_package(GTSAMCMakeTools)
     include(FindOpenMP)
     if(OPENMP_FOUND)
          set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
          set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
     endif()
endif(WITH_OPENMP)

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

if(WITH_LIBREALSENSE)
     find_package(realsense2 REQUIRED)
     if(NOT realsense2_FOUND)
          message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
     endif()
     list(APPEND EXTERNAL_PACKAGE_INCLUDES "${realsense2_INCLUDE_DIR}")
     list(APPEND EXTERNAL_PACKAGE_LIBRARIES "${realsense2_LIBRARY}")
endif(WITH_LIBREALSENSE)

# Matplotlib-cpp
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
     ${EXTERNAL_PACKAGE_LIBRARIES}
     ${PYTHON_LIBRARIES}
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
export(
     TARGETS ${RoboCommander_LIBRARIES}
     FILE "${PROJECT_BINARY_DIR}/RoboCommanderTargets.cmake"
)
export(PACKAGE RoboCommander)

# Create the RoboCommanderConfig.cmake and RoboCommanderConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
file(RELATIVE_PATH REL_LIB_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_LIB_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}" "${INC_DEST_DIR}" "${PRJOECT_LIB_DEST}")
set(CONF_DEST_DIRS "${EXEC_DEST_DIR}" "${INC_DEST_DIR}" "${PRJOECT_LIB_DEST}" "${PRJOECT_CMAKE_DEST}")

set(CONF_LOCAL_SRC_ROOT "${PROJECT_SOURCE_DIR}")
set(CONF_INC_DEST_DIR "${CMAKE_INSTALL_PREFIX}/${INC_DEST_DIR}")
set(CONF_LIB_DEST_DIR "${CMAKE_INSTALL_PREFIX}/${PRJOECT_LIB_DEST}")
set(CONF_CMAKE_DEST_DIR "${CMAKE_INSTALL_PREFIX}/${PRJOECT_CMAKE_DEST}")
set(CONF_EXEC_DEST_DIR "${CMAKE_INSTALL_PREFIX}/${EXEC_DEST_DIR}")
set(CONF_LIBRARIES "${RoboCommander_LIBRARIES}")
set(CONF_HEADERS "${RoboCommander_HEADERS}")
set(CONF_INCLUDES "${RoboCommander_INCLUDE_DIRS}")
configure_file(cmake/RoboCommanderConfig.cmake.in "${PROJECT_BINARY_DIR}/RoboCommanderConfig.cmake" @ONLY)

# ... for the install tree
# set(CONF_INCLUDE_DIRS "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_INCLUDE_DIRS "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}/RoboCommander")
# set(CONF_LIBRARIES "\${RoboCommander_CMAKE_DIR}/${REL_LIB_DIR}/RoboCommander")
# set(CONF_HEADERS "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_INCLUDES "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}/RoboCommander")
# set(CONF_LOCAL_SRC_ROOT "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_DEST_DIRS "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_CMAKE_DEST_DIR "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_LIB_DEST_DIR "\${RoboCommander_CMAKE_DIR}/${REL_LIB_DIR}/RoboCommander")
# set(CONF_INC_DEST_DIR "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# set(CONF_EXEC_DEST_DIR "\${RoboCommander_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/RoboCommanderConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RoboCommanderConfig.cmake" @ONLY)

# ... for both
configure_file(cmake/RoboCommanderConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/RoboCommanderConfigVersion.cmake" @ONLY)

# Install the RoboCommanderConfig.cmake and RoboCommanderConfigVersion.cmake
install(
     FILES
          "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RoboCommanderConfig.cmake"
          "${PROJECT_BINARY_DIR}/RoboCommanderConfigVersion.cmake"
     DESTINATION "${PRJOECT_CMAKE_DEST}"
)

install(EXPORT RoboCommanderTargets
     DESTINATION "${PRJOECT_CMAKE_DEST}"
)
# -----------------------------------------------------------------------------

# message(STATUS "REL_INCLUDE_DIR ------ ${REL_INCLUDE_DIR}")
# message(STATUS "REL_LIB_DIR ------ ${REL_LIB_DIR}")
# message(STATUS "PROJECT_SOURCE_DIR ------ ${PROJECT_SOURCE_DIR}")
# message(STATUS "PROJECT_BINARY_DIR ------ ${PROJECT_BINARY_DIR}")
# message(STATUS "REL_INCLUDE_DIR ------ ${REL_INCLUDE_DIR}")
# message(STATUS "INSTALL_LIB_DIR ------ ${INSTALL_LIB_DIR}")
# message(STATUS "PRJOECT_CMAKE_DEST ------ ${PRJOECT_CMAKE_DEST}")
# message(STATUS "CMAKE_FILES_DIRECTORY ------ ${CMAKE_FILES_DIRECTORY}")
# message(STATUS "CONF_INCLUDE_DIRS ------ ${CONF_INCLUDE_DIRS}")
# message(STATUS "INSTALL_INCLUDE_DIR ------ ${INSTALL_INCLUDE_DIR}")
# message(STATUS "RoboCommander_CMAKE_DIR ------ ${RoboCommander_CMAKE_DIR}")



# -----------------------------------------------------------------------------
# message(STATUS "Headers to install ------")
# foreach(dir ${RoboCommander_HEADERS})
#      message(STATUS "         included='${dir}'")
# endforeach()
#
# message(STATUS "Libraries to install ------")
# foreach(dir ${RoboCommander_LIBRARIES})
#      message(STATUS "         included='${dir}'")
# endforeach()
#
# message(STATUS "Header Directories to install ------")
# foreach(dir ${RoboCommander_INCLUDE_DIRS})
#      message(STATUS "         included='${dir}'")
# endforeach()

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
