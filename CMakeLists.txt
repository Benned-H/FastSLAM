# project language setup
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # generate compile_commands.json

# ---- Project ----
project(
  FastSLAM
  LANGUAGES CXX
)

# configure output files in build dir
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# library configs
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package (glog 0.4.0 REQUIRED)
find_package (gflags REQUIRED)

# testing set up
option(BUILD_TESTS "Build Tests" OFF)
option(USE_MOCK "Use Mocked Robot" OFF)
option(LM_CLEANUP "Enable landmark cleaning" OFF)
if(BUILD_TESTS)
    find_package(Catch2 3 REQUIRED)
    include(CTest)
    include(Catch)
    enable_testing()
endif()

# source configuration
add_subdirectory(src)
