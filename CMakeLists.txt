cmake_minimum_required(VERSION 3.15)
project(isotp-socket-bridge C)

set(CMAKE_C_STANDARD 99)

add_executable(isotp-socket-bridge src/main.c src/isotp/isotp.c)
