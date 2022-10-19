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
//#include "observation/Go1HardwareObservation.hpp"
#include "Go1CtrlStates.hpp"
#include "Go1Params.hpp"
#include "utils/Utils.hpp"
#include "utils/filter.hpp"
#include "EKF/Go1BasicEKF.hpp"
#include "legKinematics/Go1Kinematics.hpp"

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

  bool advance();

  bool send_cmd();

  void send_obs(Eigen::VectorXf &obs);

  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

  void updateObservation();

  void updateMovementMode();


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
//  // go1 hardware reading thread
//  std::thread thread_;
//  bool destruct = false;

  void udp_init_send();

  void receive_low_state();

  // go1 hardware switch foot order
  Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
  Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

  ros::Publisher pub_joint_cmd;
  ros::Publisher pub_joint_angle;
  ros::Publisher pub_imu;
  sensor_msgs::JointState joint_foot_msg;
  sensor_msgs::Imu imu_msg;
  ros::Subscriber sub_joy_msg;

  //! observations & actions for RL controller
  // ! observation
  size_t obDim_;
  Eigen::VectorXd obDouble_, obScaled_;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;

  Eigen::Vector3d linVelScale_, angVelScale_, gravityScale_, commandScale_;
  Eigen::VectorXd dofPosScale_, dofVelScale_;
  Eigen::VectorXd scaleFactor_;
  double clipObs_ = 100.;

  // ! action
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

//  //! observation
//  std::unique_ptr<Go1HardwareObservation> go1Obs_;

  double clipAction_ = 100.;
  double actionScale_ = 0.25;
  double stiffness_ = 20.;
  double damping_ = 0.5;

  // variables related to control and estimation
  Go1Kinematics go1_kin;
  Go1CtrlStates go1_ctrl_states;
  Go1BasicEKF go1_estimate;

  // joystic command
  double joy_cmd_velx = 0.0;
  double joy_cmd_vely = 0.0;
  double joy_cmd_velz = 0.0;
  double joy_cmd_roll_rate = 0.0;
  double joy_cmd_pitch_rate = 0.0;
  double joy_cmd_yaw_rate = 0.0;
  double joy_cmd_pitch_ang = 0.0;
  double joy_cmd_roll_ang = 0.0;
  double joy_cmd_body_height = 0.17;

  //  0 is standing, 1 is walking
  int joy_cmd_ctrl_state = 0;
  bool joy_cmd_ctrl_state_change_request = false;
  int prev_joy_cmd_ctrl_state = 0;
  bool joy_cmd_exit = false;

  // following cade is also in VILEOM
  // add leg kinematics
  // the leg kinematics is relative to body frame, which is the center of the robot
  // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
  Eigen::Vector3d p_br;
  Eigen::Matrix3d R_br;
  // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
  double leg_offset_x[4] = {};
  double leg_offset_y[4] = {};
  // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
  double motor_offset[4] = {};
  double upper_leg_length[4] = {};
  double lower_leg_length[4] = {};
  std::vector<Eigen::VectorXd> rho_fix_list;
  std::vector<Eigen::VectorXd> rho_opt_list;

  // go1 hardware foot force filter
  Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
  Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
  Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;


  // go1 hardware reading thread
  std::thread thread_;
  bool destruct = false;

};

#endif //GO1_CPP_HARDWAREGO1ROS_H
