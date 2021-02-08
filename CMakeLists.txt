cmake_minimum_required(VERSION 3.5 )
project(TestZoon)

set(CMAKE_CXX_STANDARD 14)

add_executable(TestZoon main.cpp)

target_link_libraries(TestZoon   pthread )
