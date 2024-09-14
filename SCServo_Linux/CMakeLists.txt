cmake_Minimum_required(VERSION 2.8.3)
set(project "SCServo")
project(${project})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB hdrs *.h)
file(GLOB srs *.cpp)

add_library(${project} STATIC ${hdrs} ${srs})
#add_executable(${project} main.cpp ${hdrs} ${srs})