cmake_minimum_required(VERSION 3.16)
project(go1_rl_ctrl_cpp)

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

# to find osqp.h
include_directories($ENV{UNITREE_LOCAL_INSTALL}/include/osqp)
find_package(OsqpEigen REQUIRED)

# add unitree hardware library, these flags must be correctly set
include_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib/cpp/amd64)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

# Declare cpp libraries
# common robot controller every type of robots need
add_library(go1_lib
        src/legKinematics/Go1Kinematics.hpp
        src/legKinematics/Go1Kinematics.cpp
        src/Go1Params.hpp
        src/Go1CtrlStates.hpp
        src/utils/Utils.hpp
        src/utils/Utils.cpp
        src/EKF/Go1BasicEKF.hpp
        src/EKF/Go1BasicEKF.cpp
        src/observation/Go1Observation.hpp
        src/observation/Go1HardwareObservation.hpp
        src/SwitchController.hpp)
target_link_libraries(go1_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

#target_link_libraries(${PACKAGE_NAME} /usr/lib/x86_64-linux-gnu/libtinyxml2.so)

############################
## Link C++ torch library ##
############################
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/libtorch/libtorch")

find_package(Torch REQUIRED)

add_library(torch_eigen STATIC src/torch_eigen/TorchEigen.cpp)
target_link_libraries(torch_eigen "${TORCH_LIBRARIES}")
set_property(TARGET torch_eigen PROPERTY CXX_STANDARD 14)





# lib of different types of robots
# gazebo
add_library(gazebo_go1_lib
        src/Go1RLController.hpp
        src/Go1RLController.cpp
        )
target_link_libraries(gazebo_go1_lib
        go1_lib
        torch_eigen
        libyaml-cpp.so
        )

# hardware
add_library(hardware_go1_lib
        src/Go1RLHardwareController.hpp
        src/Go1RLHardwareController.cpp
        )
target_link_libraries(hardware_go1_lib
        go1_lib
        torch_eigen
        libyaml-cpp.so
        )

# lib of different types of robots
# gazebo (servo_stand)
add_library(gazebo_servo_lib
        src/servo_stand_policy/GazeboServo.hpp
        src/servo_stand_policy/GazeboServo.cpp
        )

# hardware (servo_stand)
add_library(hardware_servo_lib
        src/servo_stand_policy/HardwareServo.hpp
        src/servo_stand_policy/HardwareServo.cpp
        )
target_link_libraries(hardware_servo_lib ${EXTRA_LIBS})

# hardware (servo_stand_switch) for test
add_library(hardware_servo_switch_lib
        src/servo_stand_policy/HardwareServoSwitch.hpp
        src/servo_stand_policy/HardwareServoSwitch.cpp
        )
target_link_libraries(hardware_servo_switch_lib ${EXTRA_LIBS})

# Declare a cpp executable for gazebo robot
# gazebo
add_executable(gazebo_go1_ctrl src/MainGazebo.cpp)
target_link_libraries(gazebo_go1_ctrl
        ${catkin_LIBRARIES}
        ${TORCH_LIBRARIES}
        gazebo_go1_lib
        gazebo_servo_lib
        )


# ************************** DEBUG **************************
# Declare a cpp executable for hardware robot
add_executable(hardware_go1_ctrl src/MainHardware.cpp)
target_link_libraries(hardware_go1_ctrl
        hardware_servo_lib
        hardware_servo_switch_lib
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )

#add_executable(stand_servo src/servo_stand_policy/servo.cpp)
#target_link_libraries(stand_servo
#        gazebo_servo_lib
#        ${catkin_LIBRARIES}
#        )





