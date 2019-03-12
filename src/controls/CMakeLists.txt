cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING CONTROLS DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/controls controls)

# PID
add_library(pid SHARED pid.cpp)
target_link_libraries(pid ${LIBS})

# Pure Pursuit
find_package(PythonLibs 2.7)
include_directories(
	${PYTHON_INCLUDE_DIRS}
	/usr/local/lib/python2.7/dist-packages/numpy/core/include
)
add_library(purepursuit SHARED waypoint_follower.cpp)
target_link_libraries(purepursuit ${LIBS} python2.7)

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     # PID
     # include_directories(${TEST_DIR}/controls)
     # add_executable(test_pid ${TEST_DIR}/controls/test_pid.cpp)
     # target_link_libraries(test_pid pid dcmotor imu)

     # Camera Gimbal Angle Controller
     include_directories(${TEST_DIR}/controls)
     add_executable(test_camera_gimbal ${TEST_DIR}/controls/test_camera_gimbal.cpp)
     target_link_libraries(test_camera_gimbal pid pca9685 i2c utils bno055)

     # Pure Pursuit
     add_executable(test_purepursuit ${TEST_DIR}/controls/test_waypoint_follower.cpp)
     target_link_libraries(test_purepursuit purepursuit)

endif(BUILD_TESTS)
