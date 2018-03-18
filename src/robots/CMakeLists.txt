cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING ROBOT CONTROL PROFILES")
endif(VERBOSE)

include_directories(${INCS_DIR}/robots robots)

# 4-wheel drive robot using two roboclaws
add_library(swansonV2 SHARED swansonV2.cpp)
target_link_libraries(swansonV2 dualclaw udp imu utils ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/robots)

     # DualClaw
     add_executable(test_swansonV2 ${TEST_DIR}/robots/test_swansonV2.cpp)
     target_link_libraries(test_swansonV2 swansonV2)

endif(BUILD_TESTS)
