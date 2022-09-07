//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &nh) {
  nh_ = nh;
  ros::param::get("package_dir", pkgDir_);
  ros::param::get("weights", ctrlWeights_);

  // ROS publisher
  pub_joint_cmd_[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_hip_controller/command", 1);
  pub_joint_cmd_[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_thigh_controller/command", 1);
  pub_joint_cmd_[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_calf_controller/command", 1);

  pub_joint_cmd_[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_hip_controller/command", 1);
  pub_joint_cmd_[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_thigh_controller/command", 1);
  pub_joint_cmd_[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_calf_controller/command", 1);

  pub_joint_cmd_[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_hip_controller/command", 1);
  pub_joint_cmd_[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_thigh_controller/command", 1);
  pub_joint_cmd_[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_calf_controller/command", 1);

  pub_joint_cmd_[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_hip_controller/command", 1);
  pub_joint_cmd_[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_thigh_controller/command", 1);
  pub_joint_cmd_[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_calf_controller/command", 1);

  actionDouble_.setZero(12);  prevActionDouble_.setZero(12);  action_.setZero(12);
  torques_.setZero(12);
  // observation
  go1Obs_ = std::make_unique<Go1Observation>(nh);

}

bool Go1RLController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");

  //! load NN parameter
  loadNNparams();

  return true;
}

void Go1RLController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLController::loadNNparams] load policy weights ");

}

bool Go1RLController::advance(double dt) {
  // updata obs except for actions
  go1Obs_->updateObservation(dt);
  Eigen::VectorXd proprioObs = go1Obs_->getObservation(); // dim=36

  // put in action
  Eigen::VectorXf obs(proprioObs.size() + 12); // full observation; dim=48
  obs << proprioObs.cast<float>(), prevActionDouble_.cast<float>();
  policy_.run(obs, action_);
  actionDouble_ = action_.cast<double>();

  // add clip to the action(should be of little use)
  actionDouble_ = actionDouble_.cwiseMin(clipAction_).cwiseMax(-clipAction_);

  prevActionDouble_ = actionDouble_;
//  std::cout << actionDouble_ << std::endl;

  // compute joint torque (PD controller)
  Eigen::VectorXd actionScaled = actionDouble_ * actionScale_;
  // pos: proprioObs.segment(12, 12); vel: proprioObs.segment(24, 12)
  torques_ = stiffness_ * (actionScaled - proprioObs.segment(12, 12)) - damping_ * proprioObs.segment(24, 12);

  return true;

}

bool Go1RLController::send_cmd() {
//  _root_control.compute_joint_torques(a1_ctrl_states);

  // send control cmd to robot via ros topic
  unitree_legged_msgs::LowCmd low_cmd;

  for (int i = 0; i < 12; i++) {
    low_cmd.motorCmd[i].mode = 0x0A;
    low_cmd.motorCmd[i].q = 0;
    low_cmd.motorCmd[i].dq = 0;
    low_cmd.motorCmd[i].Kp = 0;
    low_cmd.motorCmd[i].Kd = 0;
    low_cmd.motorCmd[i].tau = torques_[i];
    pub_joint_cmd_[i].publish(low_cmd.motorCmd[i]); // please note the joint order, should be consistent with the issac gym
  }

  return true;
}

