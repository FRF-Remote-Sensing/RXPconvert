#
# CMake file used to generate build files for the RXPconvert
# program. You'll need to run CMake with the -DCMAKE_PREFIX_PATH
# flag pointing to the location of the downloaded RiVLib, e.g.:
#
#   -DCMAKE_PREFIX_PATH=/home/tristan/Downloads/Software/RiVLIB/rivlib-2_0_1-x86-linux-gcc44/
#


# Set the project name
project(RXPconvert)

# CMake minimum version
cmake_minimum_required(VERSION 2.8)

# Make sure we're using C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Find RiVLib packages
find_package(RiVLib COMPONENTS scanlib ctrllib REQUIRED)

# Include the RiVLib include directories
include_directories(${RiVLib_INCLUDE_DIRS})

# Add the RXPconvert executable
add_executable(RXPconvert RXPconvert.cpp)

# 32-bit
set_target_properties(RXPconvert PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")

# Set link libraries
target_link_libraries(RXPconvert ${RiVLib_CTRLLIB_LIBRARY} ${RiVLib_SCANLIB_LIBRARY})
