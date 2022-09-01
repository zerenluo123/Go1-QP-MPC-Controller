//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &_nh) { }


bool Go1RLController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");

  //! load parameter
  loadParameters();
  ROS_INFO_STREAM("[Go1RLController::create] load param set");

  //! initialize observations & actions

  return true;

}

void Go1RLController::loadParameters() {
  // use absoulte path
  yamlNode_ = YAML::LoadFile("/home/zerenluo/unitree_ros_ws/src/Go1-QP-MPC-Controller/src/go1_rl_ctrl_cpp/config/parameters.yaml");

  ctrlWeights_ = yamlNode_["weights"].as<std::string>();
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");




};
