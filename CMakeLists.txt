cmake_minimum_required(VERSION 3.0)
project(simulator_prediction)

# set(CMAKE_BUILD_TYPE Release)

SET(CMAKE_BUILD_TYPE "Debug")
add_definitions("-Wall -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fsanitize=leak -fno-omit-frame-pointer")

# ggdb3-can list the origional file
set(CMAKE_C_COMPILER "gcc-9.4.0")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb3")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17) 


# Eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# yaml
find_package(yaml-cpp REQUIRED)
include_directories("/usr/local/include/yaml-cpp")

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# json
find_package(PkgConfig REQUIRED)
include_directories("/usr/include/jsoncpp")

# gtsam
find_package(GTSAM REQUIRED)
include_directories("/usr/local/include/gtsam")

# boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# threads
find_package(Threads)

set(THIRD_PARTY_LIBS
        /usr/local/lib/libyaml-cpp.a
        /usr/lib/x86_64-linux-gnu/libjsoncpp.so
        glog::glog
        gflags
        ${OpenCV_LIBS}
        ${GTSAM_LIBRARY_DIRS}
        ${CHOLMOD_LIBRARIES}
        gtsam
        ${GTEST_MAIN_LIBRARIES}
        ${Boost_LIBRARIES}
        ${JSONCPP_LIBRARIES}
)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common/spline
)

file(GLOB SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib) #lib.so文件


add_library(feature src/lanefeature.cc)
target_link_libraries(feature
        lib_ctrl_points
        lib_catmull_splinelist
        ${SOURCES}
        ${THIRD_PARTY_LIBS}
)

add_executable(test_lanes_model sim_model.cc)

target_link_libraries(test_lanes_model
        ${THIRD_PARTY_LIBS}
        file_config
        file_io
        feature
        matcher
        opter
        lib_kdtree
        lib_ctrl_points
        lib_kmmatcher
        simer
)

add_subdirectory(config)
add_subdirectory(common)
