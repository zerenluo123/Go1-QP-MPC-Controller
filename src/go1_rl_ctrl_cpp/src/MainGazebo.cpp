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
  std::unique_ptr<Go1RLController> go1_rl = std::make_unique<Go1RLController>(nh);
  go1_rl->create(0.1);



//  int stateDimension = 48; // rl_policy_module.py
//
//  torch::manual_seed(0);
//
//  torch::jit::script::Module studentModule;
//
//  studentModule = torch::jit::load("/home/zerenluo/unitree_ros_ws/src/Go1-QP-MPC-Controller/src/go1_rl_ctrl_cpp/src/cpp_model.pt"); // rl_policy_module.py
//
//  studentModule.to(torch::kCPU);
//  studentModule.eval();
//  torch::NoGradGuard no_grad_;
//
//  std::vector<torch::jit::IValue> T;
//  T.push_back(torch::ones({stateDimension}));
//
//  auto actionTensor = studentModule.forward(T).toTensor();
//
//  std::cout << actionTensor << std::endl;
}
