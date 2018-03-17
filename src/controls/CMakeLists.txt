cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING CONTROLS DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/controls controls)

# PID
add_library(pid SHARED pid.cpp)
target_link_libraries(pid utils ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     # PID
     include_directories(${TEST_DIR}/controls)
     add_executable(test_pid ${TEST_DIR}/controls/test_pid.cpp)
     target_link_libraries(test_pid pid dcmotor imu)

endif(BUILD_TESTS)
