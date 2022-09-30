//
// Created by zerenluo on 29/9/22.
//

#ifndef GO1_CPP_HARDWAREGO1ROS_H
#define GO1_CPP_HARDWAREGO1ROS_H

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "torch_eigen/TorchEigen.hpp"

// control parameters
#include "Go1Params.hpp"
#include "observation/Go1HardwareObservation.hpp"
#include "Go1CtrlStates.hpp"

// go1 hardware
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#define FOOT_FILTER_WINDOW_SIZE 5

// one of the most important thing: Go1 hardware return info with FR, FL, RR, RL order and receives info in this order
// we need to take of this order in this function
class Go1RLHardwareController {
 public:
  Go1RLHardwareController(ros::NodeHandle &nh);

  ~Go1RLHardwareController() {
    destruct = true;
    thread_.join();
  }

  bool create(double dt);

  bool advance(double dt);

  bool send_cmd();

  void send_obs(Eigen::VectorXf &obs);

 private:
  ros::NodeHandle nh_;
  std::string pkgDir_;

  // debug estimated position
  ros::Publisher pub_estimated_pose;

  // go1 hardware
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::LowState state = {0};
  UNITREE_LEGGED_SDK::LowCmd cmd = {0};
  // go1 hardware reading thread
  std::thread thread_;
  bool destruct = false;

  void udp_init_send();

  void receive_low_state();

  // go1 hardware switch foot order
  Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
  Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;


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
  std::unique_ptr<Go1HardwareObservation> go1Obs_;

  double clipAction_ = 100.;
  double actionScale_ = 0.25;
  double stiffness_ = 20.;
  double damping_ = 0.5;

  // variables related to control and estimation
  Go1CtrlStates go1_ctrl_states;


};

#endif //GO1_CPP_HARDWAREGO1ROS_H
