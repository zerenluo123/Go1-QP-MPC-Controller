//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &nh) {
  nh_ = nh;
  ros::param::get("package_dir", pkgDir_);
  ros::param::get("weights", ctrlWeights_);
}


bool Go1RLController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");

  //! load NN parameter
  loadNNparams();

  //! initialize observations & actions

  return true;

}

void Go1RLController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLController::loadNNparams] load policy weights ");

  // test for loading
  auto input = Eigen::VectorXf::Ones(48);
  Eigen::VectorXf output;
  policy_.run(input, output);
  std::cout << output << std::endl;

}
