# Root CMake for ESP-IDF project. Do not add app sources here; they live in main/.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(edge_vision_node)

idf_build_set_property(COMPILE_DEFINITIONS "-DMALLOC_CAP_SIMD=MALLOC_CAP_8BIT" APPEND)
