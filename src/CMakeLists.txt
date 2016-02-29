cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")

find_package(Boost 1.54.0 COMPONENTS python)
include_directories(${Boost_INCLUDE_DIRS} "/usr/include/python2.7")
add_library(geometry_cpp SHARED geometry.cpp)
set_target_properties(geometry_cpp PROPERTIES PREFIX "")
target_link_libraries(geometry_cpp ${Boost_LIBRARIES})
