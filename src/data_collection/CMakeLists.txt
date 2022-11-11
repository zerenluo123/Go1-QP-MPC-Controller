cmake_minimum_required(VERSION 3.16)
project(data_collection)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "-fPIC")
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


# hardware (servo_stand)
add_library(lowlevel_lib
        src/Lowlevel.hpp
        src/Lowlevel.cpp
        )
target_link_libraries(lowlevel_lib ${EXTRA_LIBS})

# hardware (sine signal)
add_library(highlevel_lib
        src/HighLevel.hpp
        src/HighLevel.cpp
        )
target_link_libraries(highlevel_lib ${EXTRA_LIBS})

# Declare a cpp executable for hardware robot
add_executable(lowlevel_demo src/MainHardware.cpp)
target_link_libraries(lowlevel_demo
        lowlevel_lib
        highlevel_lib
        ${catkin_LIBRARIES}
        )


add_executable(signal_sine src/signal_sine.cpp)
target_link_libraries(signal_sine ${EXTRA_LIBS})
