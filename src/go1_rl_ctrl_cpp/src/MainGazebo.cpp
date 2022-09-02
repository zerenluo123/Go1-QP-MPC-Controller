//
// Created by zixin on 11/1/21.
//
// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

//#include "torch_eigen/TorchEigen.hpp"
#include "Go1RLController.hpp"


int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_go1_rl_ctrl");
  ros::NodeHandle nh;

  // create go1 rl controller
  std::string pkgDir = "/home/zerenluo/unitree_ros_ws/src/Go1-QP-MPC-Controller/src/go1_rl_ctrl_cpp";
  std::unique_ptr<Go1RLController> go1_rl = std::make_unique<Go1RLController>(nh, pkgDir);
  go1_rl->create(0.1);


}
