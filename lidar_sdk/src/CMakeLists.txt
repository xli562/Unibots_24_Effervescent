include_directories(..)
include_directories(.)
aux_include_directory(. HDRS)
aux_source_directory(. SRCS)
add_to_oradar_headers(${HDRS})
add_to_oradar_sources(${SRCS})

#cmake_minimum_required(VERSION 3.4)
#project(ordlidar_bindings)
#find_package(pybind11 REQUIRED)
#pybind11_add_module(ordlidar pyordlidar.cpp)

