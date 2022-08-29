cmake_minimum_required(VERSION 3.10.1)
project(go1_rl_controller)
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

############################
## Link C++ torch library ##
############################
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/libtorch)

find_package(Torch REQUIRED)

add_library(torch_eigen STATIC src/TorchEigen.cpp)
target_link_libraries(torch_eigen "${TORCH_LIBRARIES}")
set_property(TARGET torch_eigen PROPERTY CXX_STANDARD 14)
target_include_directories(torch_eigen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


## Declare cpp libraries
## common robot controller every type of robots need
#add_library(go1_lib
#        src/Go1Params.h
#        src/Go1CtrlStates.h
#        src/utils/Utils.cpp
#        src/utils/Utils.h
#        src/Go1RobotControl.cpp
#        src/Go1RobotControl.h
#        src/Go1BasicEKF.cpp
#        src/Go1BasicEKF.h
#        )
#
## lib of different types of robots
#add_library(gazebo_go1_lib
#        src/GazeboGo1ROS.h
#        src/GazeboGo1ROS.cpp
#        )
#
#add_library(hardware_go1_lib
#        src/HardwareGo1ROS.h
#        src/HardwareGo1ROS.cpp
#        )
#
## all robots depends on the go1_lib
#target_link_libraries(gazebo_go1_lib go1_lib)
#target_link_libraries(hardware_go1_lib go1_lib ${EXTRA_LIBS})
#
## Declare a cpp executable for gazebo robot
#add_executable(gazebo_go1_ctrl src/MainGazebo.cpp)
#target_link_libraries(gazebo_go1_ctrl
#        gazebo_go1_lib
#        ${catkin_LIBRARIES}
#        )
#
## Declare a cpp executable for hardware robot
#add_executable(hardware_go1_ctrl src/MainHardware.cpp)
#target_link_libraries(hardware_go1_ctrl
#        hardware_go1_lib
#        ${catkin_LIBRARIES}
#        )
