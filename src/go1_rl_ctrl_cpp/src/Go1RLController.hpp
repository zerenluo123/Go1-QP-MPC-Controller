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

  bool advance(double dt);
  bool advance_servo(double dt);

    bool send_cmd();

  // QP-based stand policy
  bool update_foot_forces_grf(double dt);
  bool main_update(double dt);

  // debugging functions
  void send_obs(Eigen::VectorXf &obs);
  void send_foot_pos(Go1CtrlStates &go1_ctrl_states);
  void send_foot_force(Go1CtrlStates &go1_ctrl_states);

 private:
  ros::NodeHandle nh_;
  std::string pkgDir_;

  // 0,  1,  2: FL_hip, FL_thigh, FL_calf
  // 3,  4,  5: FR_hip, FR_thigh, FR_calf
  // 6,  7,  8: RL_hip, RL_thigh, RL_calf
  // 9, 10, 11: RR_hip, RR_thigh, RR_calf
  ros::Publisher pub_joint_cmd_[12];
  ros::Publisher pub_obs_;
  ros::Publisher pub_foot_pos_rel_;
  ros::Publisher pub_foot_force_;

  //! YAML parsing of parameter file
  YAML::Node yamlNode_;

  //! observations & actions
  std::unordered_map<std::string, Eigen::VectorXd> obsMap_;
  Eigen::VectorXd prevActionDouble_, actionDouble_; // double
  Eigen::VectorXf action_; // float
  Eigen::VectorXd targetPoses_;

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
  Eigen::VectorXd clipPoseLower_;
  Eigen::VectorXd clipPoseUpper_;

  double actionScale_ = 0.25;
  double stiffness_ = 18.; // 17.0
  double damping_ = 18.0; // 3.5
  double alpha_ = 0.1;
  Eigen::VectorXd pGains_;
  Eigen::VectorXd dGains_;
  std::string robot_name;

  // ! go1 control state
  Go1CtrlStates go1_ctrl_states_;

  // ! servo motion time
  double servo_motion_time_;


};
