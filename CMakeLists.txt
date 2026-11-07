cmake_minimum_required(VERSION 3.16)
project(CrowdNavigation VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(CROWD_NATIVE "Koristi -march=native" ON)
option(CROWD_OPENMP "Linkaj OpenMP (za kasnije faze)" ON)

add_library(crowd STATIC
  src/scenario.cpp
  src/grid.cpp
  src/orca.cpp
  src/sim.cpp
)
target_include_directories(crowd PUBLIC include)
target_compile_options(crowd PUBLIC -Wall -Wextra -Wpedantic)
target_compile_options(crowd PUBLIC $<$<CONFIG:Release>:-O3>)
if(CROWD_NATIVE)
  target_compile_options(crowd PUBLIC -march=native)
endif()

if(CROWD_OPENMP)
  find_package(OpenMP COMPONENTS CXX)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(crowd PUBLIC OpenMP::OpenMP_CXX)
  endif()
endif()

add_executable(crowd_bench src/main.cpp)
target_link_libraries(crowd_bench PRIVATE crowd)

enable_testing()
add_executable(test_grid tests/test_grid.cpp)
target_link_libraries(test_grid PRIVATE crowd)
add_test(NAME grid_vs_bruteforce COMMAND test_grid)

add_executable(test_sim tests/test_sim.cpp)
target_link_libraries(test_sim PRIVATE crowd)
add_test(NAME sim_neighbors COMMAND test_sim)

add_executable(test_orca tests/test_orca.cpp)
target_link_libraries(test_orca PRIVATE crowd)
add_test(NAME orca_velocity COMMAND test_orca)

option(CROWD_VIZ "Gradi vizualizator (raylib)" ON)
if(CROWD_VIZ)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(raylib)

  add_executable(crowd_viz src/viz_main.cpp)
  target_link_libraries(crowd_viz PRIVATE crowd raylib)
endif()
