//
// Created by zerenluo on 30.08.22.
//

#pragma once

#include <Eigen/Core>

// params
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <unordered_map>

#include "torch_eigen/TorchEigen.hpp"


class Go1RLController {
 public:
  Go1RLController(ros::NodeHandle &nh);

//  // ! Destructor, used to terminate map thread if there is camera
//  ~Go1RLController();

  bool create(double dt);

  bool initialize(double dt);

  bool advance(double dt);

  bool reset(double dt);

  bool preStop() { return true; };

  bool stop() { return true; };

  bool cleanup();

 private:
  ros::NodeHandle nh_;
  std::string pkgDir_;

  //! YAML parsing of parameter file
  YAML::Node yamlNode_;

  //! observations & actions
  std::unordered_map<std::string, Eigen::VectorXd> obsMap_;
  Eigen::VectorXd obDouble_, actDouble_;

  void updateObservations();

  //! controller/policy
  TorchEigen policy_;
  std::string ctrlWeights_;

  //! Parameter loading
  void loadParameters();

  //! NN parameter loading
  void loadNNparams();


};
