//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &nh, std::string& pkgDir) {
  nh_ = nh;
  pkgDir_ = pkgDir;
}


bool Go1RLController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");

  //! load parameter
  loadParameters();
  ROS_INFO_STREAM("[Go1RLController::create] load param set");

  //! load NN parameter
  loadNNparams();

  //! initialize observations & actions

  return true;

}

void Go1RLController::loadParameters() {
  // use absoulte path
  yamlNode_ = YAML::LoadFile(pkgDir_ + "/config/parameters.yaml");

  ctrlWeights_ = yamlNode_["weights"].as<std::string>();
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");

}

void Go1RLController::loadNNparams() {
  policy_.load(pkgDir_ + "/resource/cpp_model.pt");
  ROS_INFO_STREAM("\033[1;33m[Go1RLController::loadNNparams] load policy weights ");

  // test for loading
  auto input = Eigen::VectorXf::Ones(48);
  Eigen::VectorXf output;
  policy_.run(input, output);
  std::cout << output << std::endl;

}
