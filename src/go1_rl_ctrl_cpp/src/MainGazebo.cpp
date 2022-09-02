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


  // make sure the ROS infra using sim time, otherwise the controller cannot run with correct time steps
  bool use_sim_time;
  if (ros::param::get("use_sim_time", use_sim_time)) {
    if (!use_sim_time) {
      ROS_WARN_STREAM(" ROS must set use_sim_time in order to use this program! ");
      return -1;
    }
  }

  // create go1 rl controller
  std::unique_ptr<Go1RLController> go1_rl = std::make_unique<Go1RLController>(nh);
  go1_rl->create(0.1);


  return 0;

}
