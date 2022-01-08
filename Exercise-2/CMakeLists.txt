cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_2)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

# Define header and source files
set(HEADERS
    Eigen.h
    ImplicitSurface.h
    MarchingCubes.h
    Volume.h
)

set(SOURCES
    main.cpp
    Volume.cpp
)

add_executable(exercise_2 ${HEADERS} ${SOURCES})
target_include_directories(exercise_2 PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(exercise_2 Eigen3::Eigen)

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_2)
set_property(TARGET exercise_2 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
