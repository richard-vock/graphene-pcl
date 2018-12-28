cmake_minimum_required(VERSION 3.8.1)
project(graphene-pcl)
include_directories("${PROJECT_SOURCE_DIR}/include")
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_library(graphene-pcl
    "src/pcl.cpp"
    "src/pcl_cloud.cpp"
)
target_compile_features(graphene-pcl PUBLIC cxx_std_17)
target_link_libraries(graphene-pcl CONAN_PKG::graphene CONAN_PKG::CTRE "stdc++fs")
