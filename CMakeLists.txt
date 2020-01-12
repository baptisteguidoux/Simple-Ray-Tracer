cmake_minimum_required(VERSION 3.1)
project(RayTracerChallenge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/static)

# INPUTS
set(RAY_BUILD_EXAMPLES False CACHE BOOL "Build examples, default False")

SET(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_FLAGS "-Wall -g")

find_package(GTest REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_library(RayLib SHARED ${SOURCES})

# compile time definitions
add_compile_definitions(RAY_TEST_DIR="${PROJECT_SOURCE_DIR}/tests/")

install(TARGETS RayLib LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(RayLib Threads::Threads)

# Build tests using shared library
file(GLOB RAY_TEST_FILES tests/tests*.cpp)
add_executable(tests ${RAY_TEST_FILES} )
target_link_libraries(tests ${GTEST_BOTH_LIBRARIES} Threads::Threads RayLib)

# Build examples
if(${RAY_BUILD_EXAMPLES})
  
  file(GLOB RAY_EXAMPLE_FILES examples/*.cpp)
  # extract file name from full path, keep both 
  foreach(example_file_path ${RAY_EXAMPLE_FILES})
    
    get_filename_component(example_file_name ${example_file_path} NAME_WE)
    add_executable(${example_file_name} ${example_file_path})
    target_link_libraries(${example_file_name} Threads::Threads RayLib)

  endforeach()

endif()

