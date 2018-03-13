cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

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

     # # JSON Test
     # add_executable(test_json ${TEST_DIR}/utils/test_json.cpp)
     # target_link_libraries(test_json utils)
endif(BUILD_TESTS)
