cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

if(VERBOSE)
message(STATUS "------ BUILDING PROGRAMMING UTILITIES")
endif(VERBOSE)

include_directories(${INCS_DIR}/utils utils)

# Utilities
add_library(utils SHARED utils.cpp)
target_link_libraries(utils ${LIBS})

if(BUILD_TESTS)
     if(VERBOSE)
     message(STATUS "------------ Building utility tests")
     endif(VERBOSE)

     # Utilities
     include_directories(${TEST_DIR}/utils)
     add_executable(test_utils ${TEST_DIR}/utils/test_utils.cpp)
     target_link_libraries(test_utils utils)
endif(BUILD_TESTS)
