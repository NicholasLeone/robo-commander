cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING SENSOR DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/sensors sensors)

# Encoder
add_library(encoder SHARED encoder.cpp)
target_link_libraries(encoder ${LIBS})

# IMU
add_library(imu SHARED imu.cpp)
target_link_libraries(imu RTIMULib ${LIBS})

# MPU-9250
add_library(mpu9250 SHARED mpu9250.cpp)
target_link_libraries(mpu9250 utils ${LIBS})

if(BUILD_TESTS)
     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     # Encoder
     include_directories(${TEST_DIR}/sensors)
     add_executable(test_encoder ${TEST_DIR}/sensors/test_encoder.cpp)
     target_link_libraries(test_encoder encoder)

     # IMU
     add_executable(test_mpu9250 ${TEST_DIR}/sensors/test_mpu9250.cpp)
     target_link_libraries(test_mpu9250 mpu9250 serial)
endif(BUILD_TESTS)
