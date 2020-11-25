cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(robocommander)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(robocommander_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(robocommander_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(robocommander_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(robocommander_VERSION ${robocommander_VERSION_MAJOR}.${robocommander_VERSION_MINOR}.${robocommander_VERSION_PATCH} CACHE STRING "version" FORCE)

SET(PROJECT_VERSION "${robocommander_VERSION}")
SET(PROJECT_VERSION_MAJOR ${robocommander_VERSION_MAJOR})
SET(PROJECT_VERSION_MINOR ${robocommander_VERSION_MINOR})
SET(PROJECT_VERSION_PATCH ${robocommander_VERSION_PATCH})
SET(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

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
set(robocommander_CMAKE_DIR ${PRJOECT_CMAKE_DEST})

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
set(EXTRA_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

# ============================================
#       Find & Include Dependencies
# ============================================
# Find Boost Libraries
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(Threads)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

list(APPEND EXTRA_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Find OpenCV libraries
if(WITH_OPENCV)
     find_package(OpenCV 4 REQUIRED)
endif(WITH_OPENCV)

# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)
set(EXTERNAL_PACKAGE_INCLUDES "${LAPACK_INCLUDE_DIRS}" "${BLAS_INCLUDE_DIRS}" "${ARMADILLO_INCLUDE_DIRS}" "${${YAML_CPP_INCLUDE_DIR}}")
set(EXTERNAL_PACKAGE_LIBRARIES "${LAPACK_LIBRARIES}" "${BLAS_LIBRARIES}" "${ARMADILLO_LIBRARIES}" "${YAML_CPP_LIBRARIES}")

# Find library for IMU
find_library(RTIMULib libRTIMULib.so)
message(STATUS "RTIMULib: ${RTIMULib}")

# Find pigpiod libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/external/pigpiod")
find_package(pigpio REQUIRED)

# Find OpenMP libraries
if(WITH_OPENMP)
     # include(FindOpenMP)
     find_package(OpenMP)
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

if(WITH_LIBREALSENSE)
     find_package(realsense2 REQUIRED)
     if(NOT realsense2_FOUND)
          message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
     else()
          list(APPEND EXTRA_INCLUDE_DIRS ${realsense2_INCLUDE_DIR})
     endif()
endif(WITH_LIBREALSENSE)

# Matplotlib-cpp
find_package(PythonLibs 2.7 REQUIRED)
list(APPEND EXTRA_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
list(APPEND EXTRA_INCLUDE_DIRS "/usr/local/lib/python2.7/dist-packages/numpy/core/include")
set(Matplotlib_LIBS ${PYTHON_LIBRARIES})
list(APPEND Matplotlib_LIBS python2.7)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
# find_package(PCL 1.7 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common sample_consensus filters geometry)
# add_definitions(${PCL_DEFINITIONS})
if(PCL_FOUND)
     link_directories(${PCL_LIBRARY_DIRS})
     list(APPEND EXTRA_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
endif(PCL_FOUND)

include_directories(
     include
     ${CMAKE_CURRENT_BINARY_DIR}
     ${Boost_INCLUDE_DIR}
     ${OpenCV_INCLUDE_DIRS}
     ${pigpio_INCLUDE_DIR}
     ${EXTERNAL_PACKAGE_INCLUDES}
     ${EXTRA_INCLUDE_DIRS}
)

set( ${PROJECT_NAME}_EXTERNAL_LIBRARIES
     ${Boost_LIBRARIES}
     ${OpenCV_LIBRARY_DIRS}
     ${pigpiod_if2_LIBRARY}
     ${CMAKE_THREAD_LIBS_INIT}
     ${EXTERNAL_PACKAGE_LIBRARIES}
     ${Matplotlib_LIBS}
     ${OpenCV_LIBS}
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
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
     ${EXTRA_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_HEADERS
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

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -latomic")
# execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE MACHINE)
# message(STATUS "MACHINE = ${MACHINE}")
# if(${MACHINE} MATCHES "arm-linux-gnueabihf")
#      # -munaligned-access
#      set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
# elseif(${MACHINE} MATCHES "aarch64-linux-gnu")
#      set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -mstrict-align -ftree-vectorize")
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mstrict-align -ftree-vectorize")
# elseif(${MACHINE} MATCHES "powerpc64(le)?-linux-gnu")
#      set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -ftree-vectorize")
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize")
# else()
#      set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -mssse3")
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
# endif(${MACHINE} MATCHES "arm-linux-gnueabihf")

message(STATUS "Detected processor: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
     message("++ 64 bit architecture")
else()
     message("++ 32 bit architecture")
endif()

# ===========================
#     Build Targets
# ===========================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wwrite-strings")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wwrite-strings")

add_subdirectory(src)

if(BUILD_TESTS)
     add_subdirectory(tests)
endif(BUILD_TESTS)

# ===========================
#     Install Targets
# ===========================
export( TARGETS ${${PROJECT_NAME}_LIBRARIES}
        FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)
export(PACKAGE ${PROJECT_NAME})

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
set(CONF_LIBRARIES "${${PROJECT_NAME}_LIBRARIES}")
set(CONF_HEADERS "${${PROJECT_NAME}_HEADERS}")
set(CONF_INCLUDES "${${PROJECT_NAME}_INCLUDE_DIRS}")

configure_file(cmake/${PROJECT_NAME}Config.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
configure_file(cmake/${PROJECT_NAME}Config.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
configure_file(cmake/${PROJECT_NAME}ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the RoboCommanderConfig.cmake and RoboCommanderConfigVersion.cmake
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)

install(
     FILES
          "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
          "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
     DESTINATION "${PRJOECT_CMAKE_DEST}"
)

install(EXPORT ${PROJECT_NAME}Targets DESTINATION "${PRJOECT_CMAKE_DEST}")
# -----------------------------------------------------------------------------

#######################
# CPACK (Packaging)
#######################
# IF(BUILD_AS_BUNDLE)
#   INCLUDE(InstallRequiredSystemLibraries)
# ENDIF(BUILD_AS_BUNDLE)
# INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
SET(CPACK_PACKAGE_VENDOR "${PROJECT_NAME} project")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "All the tools you'd for DIY robot development.")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/Description.txt")
#SET(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME}${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
#SET(CPACK_PACKAGE_EXECUTABLES "")
#SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_PACKAGE_CONTACT "hunter.lw.young@gmail.com")
set(CPACK_INSTALL_PREFIX "/home/hunter/debs")
set(CPACK_SOURCE_IGNORE_FILES
  "\\\\.svn/"
  "\\\\.settings/"
  "${PROJECT_SOURCE_DIR}/build/[a-zA-Z0-9_]+"
  "~$"
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_PREFIX}"
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_NAME}"
  "${PROJECT_SOURCE_DIR}/bin/.*[tT]est"
  "${PROJECT_SOURCE_DIR}/bin/.*[eE]xample"
  "${PROJECT_SOURCE_DIR}/bin/.*uresourcegenerator"
  "\\\\.DS_Store"
)
SET(CPACK_GENERATOR "DEB")
SET(CPACK_SOURCE_GENERATOR "ZIP")
INCLUDE(CPack)

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
# foreach(dir ${${PROJECT_NAME}_LIBRARIES})
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
