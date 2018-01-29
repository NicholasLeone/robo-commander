cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING ROBOT CONTROL PROFILES")
endif(VERBOSE)

include_directories(${INCS_DIR}/profiles profiles)

# 4-wheel drive, app controlled robot
add_library(4WD SHARED four_wd.cpp)
target_link_libraries(4WD utils roboclaw encoder dcmotor pid udp ${LIBS})


if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/profiles)

     # 4WD
     add_executable(test_4WD ${TEST_DIR}/profiles/test_4wd.cpp)
     target_link_libraries(test_4WD 4WD)

endif(BUILD_TESTS)
