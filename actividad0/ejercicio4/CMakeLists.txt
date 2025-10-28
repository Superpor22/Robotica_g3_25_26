cmake_minimum_required(VERSION 4.0)
project(ejercicio4)

set(CMAKE_CXX_STANDARD 23)

add_definitions(-O3 -Wall -fmax-errors=1 -fno-char8_t -Wno-unknown-pragmas)

add_executable(ejercicio4 main.cpp)
