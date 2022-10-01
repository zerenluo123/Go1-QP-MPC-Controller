//
// Created by zerenluo on 30.09.22.
//

#include <Eigen/Core>
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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

// go1 hardware
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "../Go1Params.hpp"
#include "../Go1CtrlStates.hpp"
#include "../utils/Utils.hpp"
#include "../utils/filter.hpp"
#include "../EKF/Go1BasicEKF.hpp"


// TODO: compute the observation vector as the one in issac gym legged_robot.py
class Go1HardwareObservation{
 public:
  Go1HardwareObservation(ros::NodeHandle &nh):
      udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007) {
    obDim_ = 36;
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);

    // scale factor init
    linVelScale_.setZero();
    angVelScale_.setZero();
    gravityScale_.setZero();
    commandScale_.setZero();
    linVelScale_ << 2.0, 2.0, 2.0;
    angVelScale_ << 0.25, 0.25, 0.25;
    gravityScale_ << 1.0, 1.0, 1.0; // projected gravity
    commandScale_ << 2.0, 2.0, 0.25; // for cmd_velx, cmd_vely, cmd_ang_vel_yaw

    dofPosScale_.setZero(12);  dofVelScale_.setZero(12);
    for (int i = 0; i < 12; i++) {
      dofPosScale_[i] = 1.0;
      dofVelScale_[i] = 0.05;
    }
    scaleFactor_.setZero(obDim_);
    scaleFactor_ << linVelScale_, angVelScale_, gravityScale_, commandScale_, dofPosScale_, dofVelScale_;

    // ROS publisher; joint cmd(not used in shuo's case)
    pub_joint_cmd_ = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_torque_cmd", 100);

    // debug joint angle and foot force
    pub_joint_angle_ = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 100);

    // imu data
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 100);

    // joy command
    sub_joy_msg_ = nh.subscribe("/joy", 1000, &Go1HardwareObservation::joy_callback, this);

    joy_cmd_ctrl_state_ = 0;
    joy_cmd_ctrl_state_change_request_ = false;
    prev_joy_cmd_ctrl_state_ = 0;
    joy_cmd_exit_ = false;

    //init swap order, very important
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;

    // start hardware reading thread after everything initialized
    thread_ = std::thread(&Go1HardwareObservation::receive_low_state, this);

  }

  ~Go1HardwareObservation() { // move from the controller to here
    destruct = true;
    thread_.join();
  }

  void updateObservation(double dt) {
    // TODO: get all observation from measurement and concat
    // assign observation vector
    obDouble_.segment(0, 3) = go1_ctrl_states_.root_rot_mat_z.transpose() * go1_ctrl_states_.root_lin_vel; // 1. base linear velocity(robot frame)
    obDouble_.segment(3, 3) = go1_ctrl_states_.imu_ang_vel; // 2. base angular velocity(robot frame, [roll, pitch, yaw])
    obDouble_.segment(6, 3) = go1_ctrl_states_.root_rot_mat.transpose() * (-Eigen::Vector3d::UnitZ()); // 3. projected gravity(projected z unit vector)
    obDouble_.segment(9, 3) << joy_cmd_velx_, joy_cmd_vely_, joy_cmd_yaw_rate_; // 4. command([cmd_velx, cmd_vely, cmd_ang_vel_yaw])
    obDouble_.segment(12, 12) = go1_ctrl_states_.joint_pos - go1_ctrl_states_.default_joint_pos; // 5. (joint_pos - default_joint_pos)
    obDouble_.segment(24, 12) = go1_ctrl_states_.joint_vel; // 6. joint_vel
//    obDouble_.segment(36, 12) = go1_ctrl_states_.joint_actions; // 7. actions(clipped NN outputs)

    // scale the observation
    for (int i = 0; i < obDouble_.size(); i++) {
      obScaled_[i] = obDouble_[i] * scaleFactor_[i];
    }

    // clip the observation
    obScaled_ = obScaled_.cwiseMin(clipObs_).cwiseMax(-clipObs_);

    // update joy cmd
    prev_joy_cmd_ctrl_state_ = joy_cmd_ctrl_state_;

    if (joy_cmd_ctrl_state_change_request_) {
      // toggle joy_cmd_ctrl_state
      joy_cmd_ctrl_state_ = joy_cmd_ctrl_state_ + 1;
      joy_cmd_ctrl_state_ = joy_cmd_ctrl_state_ % 2; //TODO: how to toggle more states?
      joy_cmd_ctrl_state_change_request_ = false; //erase this change request;
    }

    // determine movement mode
    if (joy_cmd_ctrl_state_ == 1) {
      // walking mode, in this mode the robot should execute gait
      go1_ctrl_states_.movement_mode = 1;
    } else if (joy_cmd_ctrl_state_ == 0 && prev_joy_cmd_ctrl_state_ == 1) {
      // leave walking mode
      // lock current position, should just happen for one instance
      go1_ctrl_states_.movement_mode = 0;
    } else {
      go1_ctrl_states_.movement_mode = 0;
    }

  }


  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) { // This function applies for both gazebo and hardware
//  // left updown: change body height, not need now
//  joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
      joy_cmd_ctrl_state_change_request_ = true;
    }

    // right updown
    joy_cmd_velx_ = joy_msg->axes[4] * JOY_CMD_VELX_MAX;
    // right horiz
    joy_cmd_vely_ = joy_msg->axes[3] * JOY_CMD_VELY_MAX;
    // left horiz
    joy_cmd_yaw_rate_ = joy_msg->axes[0] * JOY_CMD_YAW_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
      std::cout << "You have pressed the exit button!!!!" << std::endl;
      joy_cmd_exit_ = true;
    }
  }

  void receive_low_state() {
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    while (destruct == false) {
//        std::cout << "OBSERVE THREAD: delta time is:"  << std::setprecision(10) << dt.toSec() << std::endl;
//         std::cout << udp.targetIP << std::endl;
      udp.Recv();
//         std::cout << "receive" << std::endl;
      udp.GetRecv(state);
//         std::cout << state.motorState[0].q << std::endl;
//         std::cout << state.imu.accelerometer[0] << std::endl;

      // fill data to go1_ctrl_states_, notice the order in state is FR, FL, RR, RL
      // fill data to go1_ctrl_states_, notice the order in go1_ctrl_states_ is FL, FR, RL, RR
      go1_ctrl_states_.root_quat = Eigen::Quaterniond(state.imu.quaternion[0],
                                                     state.imu.quaternion[1],
                                                     state.imu.quaternion[2],
                                                     state.imu.quaternion[3]);
      go1_ctrl_states_.root_rot_mat = go1_ctrl_states_.root_quat.toRotationMatrix();
      go1_ctrl_states_.root_euler = Utils::quat_to_euler(go1_ctrl_states_.root_quat);
      double yaw_angle = go1_ctrl_states_.root_euler[2];

      go1_ctrl_states_.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
      // go1_ctrl_states_.root_pos     | do not fill
      // go1_ctrl_states_.root_lin_vel | do not fill

      go1_ctrl_states_.imu_acc = Eigen::Vector3d(state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
      go1_ctrl_states_.imu_ang_vel = Eigen::Vector3d(state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
      go1_ctrl_states_.root_ang_vel = go1_ctrl_states_.root_rot_mat * go1_ctrl_states_.imu_ang_vel;

      // joint states
      for (int i = 0; i < NUM_DOF; ++i) {
        int swap_i = swap_joint_indices(i);
        go1_ctrl_states_.joint_vel[i] = state.motorState[swap_i].dq;
        // go1_ctrl_states_.joint_vel[i] = (state.motorState[swap_i].q - go1_ctrl_states_.joint_pos[i])/dt_s;
        go1_ctrl_states_.joint_pos[i] = state.motorState[swap_i].q;
      }

      // publish joint angle
      for (int i = 0; i < NUM_DOF; ++i) {
        joint_foot_msg_.position[i] = go1_ctrl_states_.joint_pos[i];
        joint_foot_msg_.velocity[i] = go1_ctrl_states_.joint_vel[i];
      }
      joint_foot_msg_.header.stamp = ros::Time::now();
      pub_joint_angle_.publish(joint_foot_msg_);

      imu_msg_.header.stamp = ros::Time::now();
      imu_msg_.angular_velocity.x = state.imu.gyroscope[0];
      imu_msg_.angular_velocity.y = state.imu.gyroscope[1];
      imu_msg_.angular_velocity.z = state.imu.gyroscope[2];

      imu_msg_.linear_acceleration.x = state.imu.accelerometer[0];
      imu_msg_.linear_acceleration.y = state.imu.accelerometer[1];
      imu_msg_.linear_acceleration.z = state.imu.accelerometer[2];
      pub_imu_.publish(imu_msg_);

      // sleep for interval_ms
      double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
      double interval_time = interval_ms / 1000.0;
      ros::Duration(interval_time).sleep();
    };
  }

  Eigen::VectorXd getObservation() { return obScaled_; }


  Go1CtrlStates getCtrlState() { return go1_ctrl_states_; }



 private:
  // 0,  1,  2: FL_hip, FL_thigh, FL_calf
  // 3,  4,  5: FR_hip, FR_thigh, FR_calf
  // 6,  7,  8: RL_hip, RL_thigh, RL_calf
  // 9, 10, 11: RR_hip, RR_thigh, RR_calf

  // 0, 1, 2, 3: FL, FR, RL, RR
  ros::Publisher pub_joint_cmd_;
  ros::Publisher pub_joint_angle_;
  ros::Publisher pub_imu_;
  sensor_msgs::JointState joint_foot_msg_;
  sensor_msgs::Imu imu_msg_;
  ros::Subscriber sub_joy_msg_;


  size_t obDim_;

  Eigen::VectorXd obDouble_, obScaled_;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;

  Eigen::Vector3d linVelScale_, angVelScale_, gravityScale_, commandScale_;
  Eigen::VectorXd dofPosScale_, dofVelScale_;
  Eigen::VectorXd scaleFactor_;
  double clipObs_ = 100.;

  Go1CtrlStates go1_ctrl_states_;
  Go1BasicEKF go1_estimate_;

  // joystick command
  double joy_cmd_velx_ = 0.0;
  double joy_cmd_vely_ = 0.0;
  double joy_cmd_velz_ = 0.0;

  double joy_cmd_yaw_rate_ = 0.0;

  double joy_cmd_body_height_ = 0.3;

  //  0 is standing, 1 is walking
  int joy_cmd_ctrl_state_ = 0;
  int prev_joy_cmd_ctrl_state_ = 0;
  bool joy_cmd_ctrl_state_change_request_ = false;
  bool joy_cmd_exit_ = false;

  // go1 hardware reading thread
  std::thread thread_;
  bool destruct = false;
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::LowState state = {0};
  Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;



};

