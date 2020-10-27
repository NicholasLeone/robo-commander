cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(robocommander)
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

set(USE_CV_VERSION       4.2 CACHE INTERNAL "Include the GTSAM library")
set(WITH_GTSAM           OFF CACHE INTERNAL "Include the GTSAM library")
set(WITH_OPENCV          ON CACHE INTERNAL "Include the OpenCV library")
set(WITH_LIBREALSENSE    ON CACHE INTERNAL "Include the librealsense2 library")
set(WITH_OPENMP          ON CACHE INTERNAL "Include the OpenMP library")
set(WITH_ARMADILLO       ON CACHE INTERNAL "Include the Armadillo library")
set(WITH_RTIMULIB        ON CACHE INTERNAL "Include the RTIMULib library")
set(WITH_PIGPIOD         ON CACHE INTERNAL "Include the pigpiod library")
set(WITH_MATPLOTLIB      ON CACHE INTERNAL "Include the matplotlib library")

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

# ============================================
#       Find & Include Dependencies
# ============================================
# Find Boost Libraries
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS system thread)

set(EXTRA_INCLUDES)
set(EXTRA_LIBRARIES)

# Find Armadillo
if(WITH_ARMADILLO)
     find_package(BLAS REQUIRED)
     find_package(LAPACK REQUIRED)
     find_package(Armadillo REQUIRED)
     find_package(PkgConfig REQUIRED)
     pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
     # set(EXTERNAL_PACKAGE_INCLUDES "${LAPACK_INCLUDE_DIRS}" "${BLAS_INCLUDE_DIRS}" "${ARMADILLO_INCLUDE_DIRS}" "${${YAML_CPP_INCLUDE_DIR}}")
     # set(EXTERNAL_PACKAGE_LIBRARIES "${LAPACK_LIBRARIES}" "${BLAS_LIBRARIES}" "${ARMADILLO_LIBRARIES}" "${YAML_CPP_LIBRARIES}")

     if(YAML_CPP_FOUND)
        message(STATUS "Found YAML_CPP: ${YAML_CPP_VERSION}")
        list(APPEND EXTRA_INCLUDES ${YAML_CPP_INCLUDE_DIR})
        list(APPEND EXTRA_LIBRARIES ${YAML_CPP_LIBRARIES})
     endif(YAML_CPP_FOUND)
     if(LAPACK_FOUND)
        message(STATUS "Found LAPACK: ${LAPACK_VERSION}")
        list(APPEND EXTRA_INCLUDES ${LAPACK_INCLUDE_DIRS})
        list(APPEND EXTRA_LIBRARIES ${LAPACK_LIBRARIES})
     endif(LAPACK_FOUND)
     if(BLAS_FOUND)
        message(STATUS "Found BLAS: ${BLAS_VERSION}")
        list(APPEND EXTRA_INCLUDES ${BLAS_INCLUDE_DIRS})
        list(APPEND EXTRA_LIBRARIES ${BLAS_LIBRARIES})
     endif(BLAS_FOUND)
     if(Armadillo_FOUND)
        message(STATUS "Found Armadillo: ${Armadillo_VERSION}")
        list(APPEND EXTRA_INCLUDES ${ARMADILLO_INCLUDE_DIRS})
        list(APPEND EXTRA_LIBRARIES ${ARMADILLO_LIBRARIES})
     endif(Armadillo_FOUND)
endif(WITH_ARMADILLO)

# Find OpenCV libraries
if(WITH_OPENCV)
     find_package(OpenCV ${USE_CV_VERSION} REQUIRED)
     if(OpenCV_FOUND)
        message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
        list(APPEND EXTRA_INCLUDES ${OpenCV_INCLUDE_DIRS})
        list(APPEND EXTRA_LIBRARIES ${OpenCV_LIBRARIES})
     endif(OpenCV_FOUND)
endif(WITH_OPENCV)

# Find library for IMU
if(WITH_RTIMULIB)
     find_library(RTIMULib libRTIMULib.so)
     # message(STATUS "RTIMULib: ${RTIMULib}")
     if(RTIMULib)
        message(STATUS "Found RTIMULib: ${RTIMULib}")
        list(APPEND EXTRA_LIBRARIES ${RTIMULib})
     endif(RTIMULib)
endif(WITH_RTIMULIB)

# Find pigpiod libraries
if(WITH_PIGPIOD)
     list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/external/pigpiod")
     find_package(pigpio REQUIRED)
     if(pigpio_FOUND)
          message(STATUS "Found pigpio: ${pigpio_VERSION}")
          list(APPEND EXTRA_INCLUDES ${pigpio_INCLUDE_DIR})
          list(APPEND EXTRA_LIBRARIES ${pigpiod_if2_LIBRARY})
     endif(pigpio_FOUND)
endif(WITH_PIGPIOD)

# Find OpenMP libraries
if(WITH_OPENMP)
     find_package(OpenMP)
     if(OPENMP_FOUND)
          message(STATUS "Found OpenMP: ${OpenMP_VERSION}")
          set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
          set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
     endif(OPENMP_FOUND)
endif(WITH_OPENMP)

# Find GTSAM libraries
if(WITH_GTSAM)
     find_package(GTSAMCMakeTools)
     find_package(GTSAM REQUIRED)
     if(GTSAM_FOUND)
          message(STATUS "Found GTSAM: ${GTSAM_VERSION}")
          list(APPEND EXTRA_INCLUDES ${GTSAM_INCLUDE_DIR})
          list(APPEND EXTRA_LIBRARIES gtsam)
     endif(GTSAM_FOUND)
     # list(APPEND EXTERNAL_PACKAGE_INCLUDES "${GTSAM_INCLUDE_DIR}")
     # list(APPEND EXTERNAL_PACKAGE_LIBRARIES gtsam)
endif(WITH_GTSAM)

if(WITH_LIBREALSENSE)
     find_package(realsense2 REQUIRED)
     if(realsense2_FOUND)
          message(STATUS "Found realsense2: ${realsense2_VERSION}")
          list(APPEND EXTRA_INCLUDES ${realsense2_INCLUDE_DIR})
     else()
          message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
     endif()
endif(WITH_LIBREALSENSE)

# Matplotlib-cpp
if(WITH_MATPLOTLIB)
     find_package(PythonLibs 2.7)
     if(PYTHONLIBS_FOUND)
          message(STATUS "Found PythonLibs: ${PYTHONLIBS_VERSION_STRING}")
          list(APPEND EXTRA_INCLUDES ${PYTHON_INCLUDE_DIRS})
          list(APPEND EXTRA_INCLUDES /usr/local/lib/python2.7/dist-packages/numpy/core/include)
          list(APPEND EXTRA_LIBRARIES ${PYTHON_LIBRARIES})
     endif(PYTHONLIBS_FOUND)
endif(WITH_MATPLOTLIB)

include_directories(
     include
     ${CMAKE_CURRENT_BINARY_DIR}
     ${Boost_INCLUDE_DIR}
     ${EXTRA_INCLUDES}
)

set( robocommander_EXTERNAL_LIBRARIES
     # ${CMAKE_THREAD_LIBS_INIT}
     ${Boost_LIBRARIES}
     ${EXTRA_LIBRARIES}
)

set( robocommander_INCLUDE_DIRS
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

file(GLOB robocommander_HEADERS
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
add_subdirectory(src)

if(BUILD_TESTS)
     add_subdirectory(tests)
endif(BUILD_TESTS)

# ===========================
#     Install Targets
# ===========================
export( TARGETS ${robocommander_LIBRARIES} FILE "${PROJECT_BINARY_DIR}/robocommanderTargets.cmake")
export(PACKAGE robocommander)

# Create the robocommanderConfig.cmake and robocommanderConfigVersion files
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
set(CONF_LIBRARIES "${robocommander_LIBRARIES}")
set(CONF_HEADERS "${robocommander_HEADERS}")
set(CONF_INCLUDES "${robocommander_INCLUDE_DIRS}")
configure_file(cmake/robocommanderConfig.cmake.in "${PROJECT_BINARY_DIR}/robocommanderConfig.cmake" @ONLY)
configure_file(cmake/robocommanderConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/robocommanderConfig.cmake" @ONLY)
configure_file(cmake/robocommanderConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/robocommanderConfigVersion.cmake" @ONLY)

# Install the robocommanderConfig.cmake and robocommanderConfigVersion.cmake
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)

install(FILES
          "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/robocommanderConfig.cmake"
          "${PROJECT_BINARY_DIR}/robocommanderConfigVersion.cmake"
     DESTINATION "${PRJOECT_CMAKE_DEST}"
)
install(EXPORT robocommanderTargets DESTINATION "${PRJOECT_CMAKE_DEST}")

#######################
# CPACK (Packaging)
#######################
# if(BUILD_AS_BUNDLE)
#   INCLUDE(InstallRequiredSystemLibraries)
# endif(BUILD_AS_BUNDLE)
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
