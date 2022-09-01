cmake_minimum_required(VERSION 3.10.1)
project(go1_rl_ctrl_cpp)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        unitree_legged_msgs
        )

find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rc_sim_oub
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# add unitree hardware library, these flags must be correctly set
include_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib/cpp/amd64)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

#target_link_libraries(${PACKAGE_NAME} /usr/lib/x86_64-linux-gnu/libtinyxml2.so)

############################
## Link C++ torch library ##
############################
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/libtorch/libtorch")

find_package(Torch REQUIRED)

add_library(torch_eigen STATIC src/torch_eigen/TorchEigen.cpp)
target_link_libraries(torch_eigen "${TORCH_LIBRARIES}")
set_property(TARGET torch_eigen PROPERTY CXX_STANDARD 14)

# Declare cpp libraries
# common robot controller every type of robots need
add_library(gazebo_go1_lib
        src/Go1RLController.hpp
        src/Go1RLController.cpp
        )
target_link_libraries(gazebo_go1_lib torch_eigen libyaml-cpp.so)


# Declare a cpp executable for gazebo robot
add_executable(gazebo_go1_ctrl src/MainGazebo.cpp)
target_link_libraries(gazebo_go1_ctrl
        ${catkin_LIBRARIES}
        ${TORCH_LIBRARIES}
        gazebo_go1_lib
        )
