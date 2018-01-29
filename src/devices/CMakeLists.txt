cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING DEVICE DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/devices devices)

# RoboClaw
add_library(roboclaw SHARED roboclaw.cpp)
target_link_libraries(roboclaw ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/devices)

     # RoboClaw
     add_executable(test_roboclaw ${TEST_DIR}/devices/test_roboclaw.cpp)
     target_link_libraries(test_roboclaw roboclaw)

endif(BUILD_TESTS)
