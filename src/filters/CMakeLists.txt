cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING FILTER CLASSES")
endif(VERBOSE)

include_directories(${INCS_DIR}/filters filters)

# EKF
add_library(ekf SHARED ekf.cpp)
target_link_libraries(ekf ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     include_directories(${TEST_DIR}/filters)

     # EKF
     add_executable(test_ekf ${TEST_DIR}/filters/test_ekf.cpp)
     target_link_libraries(test_ekf ekf)

endif(BUILD_TESTS)
