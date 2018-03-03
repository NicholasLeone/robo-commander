cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING DEVICE DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/devices devices)

# RoboClaw
add_library(roboclaw SHARED roboclaw.cpp)
target_link_libraries(roboclaw ${LIBS})

# PWM Driver - PCA-9685
add_library(pca9685 SHARED pca9685.cpp)
target_link_libraries(pca9685 ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/devices)

     # RoboClaw
     add_executable(test_roboclaw ${TEST_DIR}/devices/test_roboclaw.cpp)
     target_link_libraries(test_roboclaw roboclaw)

     # PWM Driver - PCA-9685
     add_executable(test_pca9685 ${TEST_DIR}/devices/test_pca9685.cpp)
     target_link_libraries(test_pca9685 pca9685 i2c ${LIBS})

     # LED Color Test
     add_executable(test_leds ${TEST_DIR}/devices/test_leds.cpp)
     target_link_libraries(test_leds pca9685 i2c ${LIBS})

endif(BUILD_TESTS)
