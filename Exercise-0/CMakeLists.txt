cmake_minimum_required(VERSION 3.10)

# Arbitrary name
project(cmake-tutorial)


# Add some options which appear in the GUI
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(MAGIC_TOGGLE True CACHE BOOL "Some toggable")


# Set C++ standards
set(CMAKE_CXX_STANDARD 14)

# This message will be display during the configuration of the project
if(${MAGIC_TOGGLE})
    message(STATUS "Magic Toggle is on.")
else()
    message(STATUS "Magic Toggle is off.")
endif()



# LIBRARIES

# Let's have a look how libraries are setup
LIST(APPEND LIB_FILES "lib/algo.h" "lib/algo.cpp")
add_library(lib STATIC ${LIB_FILES})



# EXECUTABLES

# Let's be explicit about the header and source files
LIST(APPEND HEADER_FILES "util.h" "constants.h")
LIST(APPEND SOURCE_FILES "main.cpp")


# Add a path to the installed Eigen library for find_package command https://cmake.org/cmake/help/latest/command/find_package.html
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to the folder holding Eigen3Config.cmake file in the Eigen installation folder")
# finds and loads settings from the installed Eigen package (sets EIGEN3_INCLUDE_DIR variable)
find_package(Eigen3 REQUIRED)


# Define an executable with a name and the files
add_executable(app ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(app PUBLIC "lib" ${EIGEN3_INCLUDE_DIR})  # Make the lib folder an include directory
target_link_libraries(app PUBLIC lib)  # Link this library to our executable
# Since Eigen is a header only library we do not need to link it, only to include
