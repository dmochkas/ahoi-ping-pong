cmake_minimum_required(VERSION 3.10)

project(ahoi-ping-pong C)

include(config.cmake)

set(AHOI_LIB "ahoi-lib")
set(CORE_LIB "ahoi-core")

add_subdirectory(src)
add_subdirectory(app)