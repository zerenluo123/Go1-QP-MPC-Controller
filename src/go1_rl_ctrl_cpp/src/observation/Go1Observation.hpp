//
// Created by zerenluo on 02.09.22.
//
#include <Eigen/Core>

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

// unitree_legged_msgs for gazebo
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/Observation.h>
#include <unitree_legged_msgs/FootPos.h>
#include <unitree_legged_msgs/FootForce.h>


#include "../Go1Params.hpp"
#include "../Go1CtrlStates.hpp"
#include "../utils/Utils.hpp"
#include "../utils/filter.hpp"
#include "../EKF/Go1BasicEKF.hpp"
#include "../legKinematics/Go1Kinematics.hpp"

// TODO: compute the observation vector as the one in issac gym legged_robot.py
class Go1Observation{
 public:
  Go1Observation(ros::NodeHandle &nh) {
    obDim_ = 36;
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);

    historyTempMem_.setZero(num_history_stack_*12);
    jointPosErrorHist_.setZero(num_history_stack_*12);
    jointVelHist_.setZero(num_history_stack_*12);

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

    // ROS register callback, call backs directly modify variables in Go1CtrlStates
    sub_gt_pose_msg_ = nh.subscribe("/torso_odom", 100, &Go1Observation::gt_pose_callback, this);
    sub_imu_msg_ = nh.subscribe("/trunk_imu", 100, &Go1Observation::imu_callback, this);

    sub_joint_msg_[0] = nh.subscribe("/go1_gazebo/FL_hip_controller/state", 2, &Go1Observation::FL_hip_state_callback, this);
    sub_joint_msg_[1] = nh.subscribe("/go1_gazebo/FL_thigh_controller/state", 2, &Go1Observation::FL_thigh_state_callback, this);
    sub_joint_msg_[2] = nh.subscribe("/go1_gazebo/FL_calf_controller/state", 2, &Go1Observation::FL_calf_state_callback, this);

    sub_joint_msg_[3] = nh.subscribe("/go1_gazebo/FR_hip_controller/state", 2, &Go1Observation::FR_hip_state_callback, this);
    sub_joint_msg_[4] = nh.subscribe("/go1_gazebo/FR_thigh_controller/state", 2, &Go1Observation::FR_thigh_state_callback, this);
    sub_joint_msg_[5] = nh.subscribe("/go1_gazebo/FR_calf_controller/state", 2, &Go1Observation::FR_calf_state_callback, this);

    sub_joint_msg_[6] = nh.subscribe("/go1_gazebo/RL_hip_controller/state", 2, &Go1Observation::RL_hip_state_callback, this);
    sub_joint_msg_[7] = nh.subscribe("/go1_gazebo/RL_thigh_controller/state", 2, &Go1Observation::RL_thigh_state_callback, this);
    sub_joint_msg_[8] = nh.subscribe("/go1_gazebo/RL_calf_controller/state", 2, &Go1Observation::RL_calf_state_callback, this);

    sub_joint_msg_[9] = nh.subscribe("/go1_gazebo/RR_hip_controller/state", 2, &Go1Observation::RR_hip_state_callback, this);
    sub_joint_msg_[10] = nh.subscribe("/go1_gazebo/RR_thigh_controller/state", 2, &Go1Observation::RR_thigh_state_callback, this);
    sub_joint_msg_[11] = nh.subscribe("/go1_gazebo/RR_calf_controller/state", 2, &Go1Observation::RR_calf_state_callback, this);

    sub_foot_contact_msg_[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &Go1Observation::FL_foot_contact_callback, this);
    sub_foot_contact_msg_[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &Go1Observation::FR_foot_contact_callback, this);
    sub_foot_contact_msg_[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &Go1Observation::RL_foot_contact_callback, this);
    sub_foot_contact_msg_[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &Go1Observation::RR_foot_contact_callback, this);

    sub_joy_msg_ = nh.subscribe("/joy", 1000, &Go1Observation::joy_callback, this);

    joy_cmd_ctrl_state_ = 0;
    joy_cmd_ctrl_state_change_request_ = false;
    prev_joy_cmd_ctrl_state_ = 0;
    joy_cmd_exit_ = false;

    go1_ctrl_states_.reset();
    go1_ctrl_states_.resetFromROSParam(nh);

    // don't know if filter is needed
    acc_x_ = MovingWindowFilter(5);
    acc_y_ = MovingWindowFilter(5);
    acc_z_ = MovingWindowFilter(5);
    gyro_x_ = MovingWindowFilter(5);
    gyro_y_ = MovingWindowFilter(5);
    gyro_z_ = MovingWindowFilter(5);

    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1881;
    leg_offset_x[1] = 0.1881;
    leg_offset_x[2] = -0.1881;
    leg_offset_x[3] = -0.1881;
    leg_offset_y[0] = 0.04675;
    leg_offset_y[1] = -0.04675;
    leg_offset_y[2] = 0.04675;
    leg_offset_y[3] = -0.04675;
    motor_offset[0] = 0.08;
    motor_offset[1] = -0.08;
    motor_offset[2] = 0.08;
    motor_offset[3] = -0.08;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.213;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.213;

    for (int i = 0; i < NUM_LEG; i++) {
      Eigen::VectorXd rho_fix(5);
      rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
      Eigen::VectorXd rho_opt(3);
      rho_opt << 0.0, 0.0, 0.0;
      rho_fix_list.push_back(rho_fix);
      rho_opt_list.push_back(rho_opt);
    }

    // start hardware reading thread after everything initialized
    thread_ = std::thread(&Go1Observation::estimation, this);

  }

  ~Go1Observation() {
    destruct_ = true;
    thread_.join();
  }

  void updateObservation(double dt) {
//    // update state estimation, include base position and base velocity(world frame)
//    if (!go1_estimate_.is_inited()) {
//      go1_estimate_.init_state(go1_ctrl_states_);
//    } else {
//      go1_estimate_.update_estimation(go1_ctrl_states_, dt);
//    }

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

    updateMovementMode();

  }

  void updateHistory() {
    // TODO: get history of pos, vel (action hist is in the RL controller)
    historyTempMem_ = jointVelHist_;
    jointVelHist_.head((num_history_stack_-1) * 12) = historyTempMem_.tail((num_history_stack_-1) * 12);
    jointVelHist_.tail(12) = go1_ctrl_states_.joint_vel;

    historyTempMem_ = jointPosErrorHist_;
    jointPosErrorHist_.head((num_history_stack_-1) * 12) = historyTempMem_.tail((num_history_stack_-1) * 12);
    jointPosErrorHist_.tail(12) = go1_ctrl_states_.joint_pos - go1_ctrl_states_.default_joint_pos;
  }

  void updateMovementMode() { // keep this: the movement mode in ctrl state is used in the EKF.
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
      go1_ctrl_states_.movement_mode = 0;
    } else {
      go1_ctrl_states_.movement_mode = 0;
    }

  } // updateMovementMode

  Eigen::VectorXd getObservation() { return obScaled_; }
  Go1CtrlStates getCtrlState() { return go1_ctrl_states_; }
  int getMovementMode() { return go1_ctrl_states_.movement_mode; }
  Eigen::Matrix<double, NUM_DOF, 1> getJointPose() { return go1_ctrl_states_.joint_pos; }


  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) { // This function applies for both gazebo and hardware
    // left updown: change body height, not need now
    joy_cmd_velz_ = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

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
    // up-down button
    joy_cmd_pitch_rate_ = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    joy_cmd_roll_rate_ = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;


    // lb
    if (joy_msg->buttons[4] == 1) {
      std::cout << "You have pressed the exit button!!!!" << std::endl;
      joy_cmd_exit_ = true;
    }
  }

  // ****************************************************************************************************************
  // ***************************************** Below is for state in gazebo *****************************************
  // ****************************************************************************************************************
  // callback functions
  void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    // update
    go1_ctrl_states_.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                  odom->pose.pose.orientation.x,
                                                  odom->pose.pose.orientation.y,
                                                  odom->pose.pose.orientation.z);
    // go1_ctrl_states_.root_pos << odom->pose.pose.position.x,
    //         odom->pose.pose.position.y,
    //         odom->pose.pose.position.z;
//     // make sure root_lin_vel is in world frame
//     go1_ctrl_states_.root_lin_vel << odom->twist.twist.linear.x,
//             odom->twist.twist.linear.y,
//             odom->twist.twist.linear.z;

    // make sure root_ang_vel is in world frame
    // go1_ctrl_states_.root_ang_vel << odom->twist.twist.angular.x,
    //         odom->twist.twist.angular.y,
    //         odom->twist.twist.angular.z;

    // calculate several useful variables
    // euler should be roll pitch yaw
    go1_ctrl_states_.root_rot_mat = go1_ctrl_states_.root_quat.toRotationMatrix();
    go1_ctrl_states_.root_euler = Utils::quat_to_euler(go1_ctrl_states_.root_quat);
    double yaw_angle = go1_ctrl_states_.root_euler[2];

    go1_ctrl_states_.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
    for (int i = 0; i < NUM_LEG; ++i) {
      go1_ctrl_states_.foot_pos_rel.block<3, 1>(0, i) = go1_kin_.fk(
          go1_ctrl_states_.joint_pos.segment<3>(3 * i),
          rho_opt_list[i], rho_fix_list[i]);
      go1_ctrl_states_.j_foot.block<3, 3>(3 * i, 3 * i) = go1_kin_.jac(
          go1_ctrl_states_.joint_pos.segment<3>(3 * i),
          rho_opt_list[i], rho_fix_list[i]);
      Eigen::Matrix3d tmp_mtx = go1_ctrl_states_.j_foot.block<3, 3>(3 * i, 3 * i);
      Eigen::Vector3d tmp_vec = go1_ctrl_states_.joint_vel.segment<3>(3 * i);
      go1_ctrl_states_.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

      go1_ctrl_states_.foot_pos_abs.block<3, 1>(0, i) = go1_ctrl_states_.root_rot_mat * go1_ctrl_states_.foot_pos_rel.block<3, 1>(0, i);
      go1_ctrl_states_.foot_vel_abs.block<3, 1>(0, i) = go1_ctrl_states_.root_rot_mat * go1_ctrl_states_.foot_vel_rel.block<3, 1>(0, i);

      go1_ctrl_states_.foot_pos_world.block<3, 1>(0, i) = go1_ctrl_states_.foot_pos_abs.block<3, 1>(0, i) + go1_ctrl_states_.root_pos;
      go1_ctrl_states_.foot_vel_world.block<3, 1>(0, i) = go1_ctrl_states_.foot_vel_abs.block<3, 1>(0, i) + go1_ctrl_states_.root_lin_vel;
    }
  }

  void imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
    // go1_ctrl_states_.root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(imu->orientation.w),
    //                                               quat_x.CalculateAverage(imu->orientation.x),
    //                                               quat_y.CalculateAverage(imu->orientation.y),
    //                                               quat_z.CalculateAverage(imu->orientation.z));
    go1_ctrl_states_.imu_acc = Eigen::Vector3d(
        acc_x_.CalculateAverage(imu->linear_acceleration.x),
        acc_y_.CalculateAverage(imu->linear_acceleration.y),
        acc_z_.CalculateAverage(imu->linear_acceleration.z)
    );
    go1_ctrl_states_.imu_ang_vel = Eigen::Vector3d(
        gyro_x_.CalculateAverage(imu->angular_velocity.x),
        gyro_y_.CalculateAverage(imu->angular_velocity.y),
        gyro_z_.CalculateAverage(imu->angular_velocity.z)
    );
    go1_ctrl_states_.root_ang_vel = go1_ctrl_states_.root_rot_mat * go1_ctrl_states_.imu_ang_vel;
  }

  // FL
  void FL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[0] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[0] = go1_joint_state.dq;
  }

  void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[1] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[1] = go1_joint_state.dq;
  }

  void FL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[2] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[2] = go1_joint_state.dq;
  }

// FR
  void FR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[3] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[3] = go1_joint_state.dq;
  }

  void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[4] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[4] = go1_joint_state.dq;
  }

  void FR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[5] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[5] = go1_joint_state.dq;
  }

// RL
  void RL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[6] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[6] = go1_joint_state.dq;
  }

  void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[7] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[7] = go1_joint_state.dq;
  }

  void RL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[8] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[8] = go1_joint_state.dq;
  }

// RR
  void RR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[9] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[9] = go1_joint_state.dq;
  }

  void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[10] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[10] = go1_joint_state.dq;
  }

  void RR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state) {
    go1_ctrl_states_.joint_pos[11] = go1_joint_state.q;
    go1_ctrl_states_.joint_vel[11] = go1_joint_state.dq;
  }

  // foot contact force
  void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states_.foot_force[0] = force.wrench.force.z;
  }

  void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states_.foot_force[1] = force.wrench.force.z;
  }

  void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states_.foot_force[2] = force.wrench.force.z;
  }

  void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states_.foot_force[3] = force.wrench.force.z;
  }


  void estimation() {

    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    while (destruct_ == false) {

      now = ros::Time::now();
      dt = now - prev;
      prev = now;
      double dt_s = dt.toSec();

      // update state estimation, include base position and base velocity(world frame)
      auto t1 = ros::Time::now();
      if (!go1_estimate_.is_inited()) {
        go1_estimate_.init_state(go1_ctrl_states_);
      } else {
        go1_estimate_.update_estimation(go1_ctrl_states_, dt_s);
      }
      auto t2 = ros::Time::now();
      ros::Duration run_dt = t2 - t1;


      double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
      // sleep for interval_ms
      double interval_time = interval_ms / 1000.0;
      if (interval_time > run_dt.toSec()) {
        ros::Duration(interval_time - run_dt.toSec()).sleep();
      }

    } // while

  } // void estimation()



 private:
  // 0,  1,  2: FL_hip, FL_thigh, FL_calf
  // 3,  4,  5: FR_hip, FR_thigh, FR_calf
  // 6,  7,  8: RL_hip, RL_thigh, RL_calf
  // 9, 10, 11: RR_hip, RR_thigh, RR_calf
  ros::Subscriber sub_joint_msg_[12];

  // 0, 1, 2, 3: FL, FR, RL, RR
  ros::Subscriber sub_foot_contact_msg_[4];
  ros::Subscriber sub_gt_pose_msg_;
  ros::Subscriber sub_imu_msg_;
  ros::Subscriber sub_joy_msg_;

  size_t obDim_;

  Eigen::VectorXd obDouble_, obScaled_;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;

  Eigen::Vector3d linVelScale_, angVelScale_, gravityScale_, commandScale_;
  Eigen::VectorXd dofPosScale_, dofVelScale_;
  Eigen::VectorXd scaleFactor_;
  double clipObs_ = 100.;

  Go1Kinematics go1_kin_;
  Go1CtrlStates go1_ctrl_states_;
  Go1BasicEKF go1_estimate_;

  // filters
  MovingWindowFilter acc_x_;
  MovingWindowFilter acc_y_;
  MovingWindowFilter acc_z_;
  MovingWindowFilter gyro_x_;
  MovingWindowFilter gyro_y_;
  MovingWindowFilter gyro_z_;

  // joystick command
  double joy_cmd_velx_ = 0.0;
  double joy_cmd_vely_ = 0.0;
  double joy_cmd_velz_ = 0.0;

  double joy_cmd_pitch_rate_ = 0.0;
  double joy_cmd_roll_rate_ = 0.0;
  double joy_cmd_yaw_rate_ = 0.0;

  double joy_cmd_body_height_ = 0.3;

  //  0 is standing, 1 is walking
  int joy_cmd_ctrl_state_ = 0;
  int prev_joy_cmd_ctrl_state_ = 0;
  bool joy_cmd_ctrl_state_change_request_ = false;
  bool joy_cmd_exit_ = false;

  // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
  double leg_offset_x[4] = {};
  double leg_offset_y[4] = {};
  // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
  double motor_offset[4] = {};
  double upper_leg_length[4] = {};
  double lower_leg_length[4] = {};
  std::vector<Eigen::VectorXd> rho_fix_list;
  std::vector<Eigen::VectorXd> rho_opt_list;

  std::thread thread_;
  bool destruct_ = false;

  // ! history of joint state
  Eigen::VectorXd historyTempMem_, jointPosErrorHist_, jointVelHist_;
  int num_history_stack_;



};
