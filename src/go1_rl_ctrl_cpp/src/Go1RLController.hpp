//
// Created by zerenluo on 30.08.22.
//

#pragma once
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

// params
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>

#include "torch_eigen/TorchEigen.hpp"

// control parameters
#include "Go1Params.hpp"
#include "observation/Go1Observation.hpp"
#include "Go1CtrlStates.hpp"


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

  // callback functions
  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

  bool send_cmd();

  void send_obs(Eigen::VectorXf &obs);



    private:
  ros::NodeHandle nh_;
  std::string pkgDir_;

  // 0,  1,  2: FL_hip, FL_thigh, FL_calf
  // 3,  4,  5: FR_hip, FR_thigh, FR_calf
  // 6,  7,  8: RL_hip, RL_thigh, RL_calf
  // 9, 10, 11: RR_hip, RR_thigh, RR_calf
  ros::Publisher pub_joint_cmd_[12];
  ros::Publisher pub_obs_;

  //! YAML parsing of parameter file
  YAML::Node yamlNode_;

  //! observations & actions
  std::unordered_map<std::string, Eigen::VectorXd> obsMap_;
  Eigen::VectorXd prevActionDouble_, actionDouble_; // double
  Eigen::VectorXf action_; // float
  Eigen::VectorXd torques_;

  //! controller/policy
  TorchEigen standPolicy_;
  TorchEigen policy_;
  std::string standCtrlWeights_;
  std::string ctrlWeights_;

  //! Parameter loading
  void loadParameters();

  //! NN parameter loading
  void loadNNparams();

  //! observation
  std::unique_ptr<Go1Observation> go1Obs_;

  double clipAction_ = 100.;
  double actionScale_ = 0.25;
  double stiffness_ = 20.;
  double damping_ = 0.5;

  // ! go1 control state
  Go1CtrlStates go1_ctrl_states_;



};
