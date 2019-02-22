#
# CMake file used to generate build files for the RXPconvert
# program. You'll need to run CMake with the -DRiVLib_DIR
# flag pointing to the location of the downloaded RiVLib, e.g.:
#
#   -DRiVLib_DIR=/home/tristan/Downloads/Software/RiVLIB/rivlib-2_5_7-x86_64-linux-gcc55/cmake
#


# Set the project name
project(RXPconvert)

# CMake minimum version
cmake_minimum_required(VERSION 2.8)

# Make sure we're using C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find RiVLib packages
find_package(RiVLib COMPONENTS scanlib REQUIRED)

# Add the RXPconvert executable
add_executable(RXPconvert RXPconvert.cpp)

# Include the RiVLib include directories
target_include_directories(RXPconvert PRIVATE ${RiVLib_INCLUDE_DIRS})

# Set link libraries
target_link_libraries(RXPconvert PRIVATE ${RiVLib_SCANLIB_LIBRARY})
