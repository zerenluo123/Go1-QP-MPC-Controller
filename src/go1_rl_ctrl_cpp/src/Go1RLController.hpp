//
// Created by zerenluo on 30.08.22.
//

#pragma once

#include <Eigen/Core>

// params
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

#include <unordered_map>

#include "torch_eigen/TorchEigen.hpp"

// control parameters
#include "Go1Params.hpp"

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


 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_joy_msg_;
  std::string pkgDir_;

  // joystick command
  double joy_cmd_velx_ = 0.0;
  double joy_cmd_vely_ = 0.0;
  double joy_cmd_velz_ = 0.0;

  double joy_cmd_yaw_rate_ = 0.0;

  double joy_cmd_body_height_ = 0.3;

  //  0 is standing, 1 is walking
  int joy_cmd_ctrl_state_ = 0;
  int prev_joy_cmd_ctrl_state_ = 0;
  bool joy_cmd_exit_ = false;

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
