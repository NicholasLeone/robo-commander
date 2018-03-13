cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING ROBOT CONTROL PROFILES")
endif(VERBOSE)

include_directories(${INCS_DIR}/profiles profiles)

# # 4-wheel drive, app controlled robot, with basic motor drive hardware
# add_library(4WD SHARED four_wd.cpp)
# target_link_libraries(4WD utils roboclaw encoder dcmotor pid udp ${LIBS})

# 4-wheel drive robot using two roboclaws
add_library(DualClaw SHARED dual_roboclaw.cpp)
target_link_libraries(DualClaw utils roboclaw ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/profiles)

     # # 4WD
     # add_executable(test_4WD ${TEST_DIR}/profiles/test_4wd.cpp)
     # target_link_libraries(test_4WD 4WD)

     # DualClaw
     add_executable(test_dualclaw ${TEST_DIR}/profiles/test_dual_roboclaw.cpp)
     target_link_libraries(test_dualclaw DualClaw)

endif(BUILD_TESTS)
