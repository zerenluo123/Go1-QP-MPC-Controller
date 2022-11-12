//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &nh) {
  nh_ = nh;
  ros::param::get("package_dir", pkgDir_);
  ros::param::get("weights", ctrlWeights_);
  ros::param::get("stand_weights", standCtrlWeights_);
  ros::param::get("stiffness", stiffness_);
  ros::param::get("damping", damping_);


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
  targetPoses_.setZero(12);  clipPoseLower_.setZero(12);  clipPoseUpper_.setZero(12);
  clipPoseLower_ << -0.9425, -0.4817, -2.6285, -0.9425, -0.4817, -2.6285, -0.9425, -0.4817, -2.6285, -0.9425, -0.4817, -2.6285;
  clipPoseUpper_ << 0.9425,  2.7855, -0.9320,  0.9425,  2.7855, -0.9320,  0.9425,  2.7855, -0.9320,  0.9425,  2.7855, -0.9320;

  // ! set kp kd gains
  pGains_.setZero(12);  dGains_.setZero(12);
  pGains_ << 20., 50., 50.,
             20., 50., 50.,
             20., 50., 50.,
             20., 50., 50.;
  dGains_ << 1., 2., 2.,
             1., 2., 2.,
             1., 2., 2.,
             1., 2., 2.;

  // observation
  go1Obs_ = std::make_unique<Go1Observation>(nh);

  // debug: publish observation, foot pos rel, foot force
  pub_obs_ = nh.advertise<unitree_legged_msgs::Observation>("/gazebo_go1/observation", 1);
  pub_foot_pos_rel_ = nh.advertise<unitree_legged_msgs::FootPos>("/gazebo_go1/foot_pos_rel", 1);
  pub_foot_force_ = nh.advertise<unitree_legged_msgs::FootForce>("/gazebo_go1/foot_force", 1);

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");
  loadNNparams();
}

void Go1RLController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << standCtrlWeights_ << "\033[0m");
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");

  standPolicy_.load(pkgDir_ + "/resource/" + standCtrlWeights_);
  ROS_INFO_STREAM("[Go1RLController::loadNNparams] load stand policy weights ");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLController::loadNNparams] load policy weights ");

}

bool Go1RLController::advance(double dt) {
  // walk
  // updata obs except for actions
  go1Obs_->updateObservation(dt);
  Eigen::VectorXd proprioObs = go1Obs_->getObservation(); // dim=36
  go1_ctrl_states_ = go1Obs_->getCtrlState();

  // put in action
  Eigen::VectorXf obs(proprioObs.size() + 12); // full observation; dim=48
  obs << proprioObs.cast<float>(), prevActionDouble_.cast<float>();

  policy_.run(obs, action_);
  actionDouble_ = action_.cast<double>();

  // add clip to the action(should be of little use)
  actionDouble_ = actionDouble_.cwiseMin(clipAction_).cwiseMax(-clipAction_);
  prevActionDouble_ = actionDouble_;

  // compute joint actions (PD controller)
  Eigen::VectorXd actionScaled = actionDouble_ * actionScale_;
  targetPoses_ = actionScaled + go1_ctrl_states_.default_joint_pos;
  // add clip to target poses
  targetPoses_ = targetPoses_.cwiseMin(clipPoseUpper_).cwiseMax(clipPoseLower_);


//  // *********** debug **********
//  send_obs(obs);
//  send_foot_pos(go1_ctrl_states_);
//  send_foot_force(go1_ctrl_states_);

  return true;

}

bool Go1RLController::send_cmd() {
  // send control cmd to robot via ros topic
  unitree_legged_msgs::LowCmd low_cmd;

  // ! assign kp kd value for corresponding joints

  for (int i = 0; i < 12; i++) {
    low_cmd.motorCmd[i].mode = 0x0A;
    low_cmd.motorCmd[i].q = targetPoses_(i);
    low_cmd.motorCmd[i].dq = 0;
    low_cmd.motorCmd[i].Kp = pGains_(i);
    low_cmd.motorCmd[i].Kd = dGains_(i);
    low_cmd.motorCmd[i].tau = 0;
    pub_joint_cmd_[i].publish(low_cmd.motorCmd[i]); // please note the joint order, should be consistent with the issac gym
  }

  return true;
}

// ***************************************************************
// ************************** Debugging **************************
// ***************************************************************
void Go1RLController::send_obs(Eigen::VectorXf &obs) {
  // publish the observation
  unitree_legged_msgs::Observation obs_msg;

  obs_msg.lin_vel_x = obs[0];
  obs_msg.lin_vel_y = obs[1];
  obs_msg.lin_vel_z = obs[2];

  obs_msg.ang_vel_x = obs[3];
  obs_msg.ang_vel_y = obs[4];
  obs_msg.ang_vel_z = obs[5];

  obs_msg.gravity_x = obs[6];
  obs_msg.gravity_y = obs[7];
  obs_msg.gravity_z = obs[8];

  obs_msg.cmd_vel_x = obs[9];
  obs_msg.cmd_vel_y = obs[10];
  obs_msg.cmd_vel_z = obs[11];

  obs_msg.pos_FL_hip = obs[12];   obs_msg.pos_FL_thigh = obs[13];   obs_msg.pos_FL_calf = obs[14];
  obs_msg.pos_FR_hip = obs[15];   obs_msg.pos_FR_thigh = obs[16];   obs_msg.pos_FR_calf = obs[17];
  obs_msg.pos_RL_hip = obs[18];   obs_msg.pos_RL_thigh = obs[19];   obs_msg.pos_RL_calf = obs[20];
  obs_msg.pos_RR_hip = obs[21];   obs_msg.pos_RR_thigh = obs[22];   obs_msg.pos_RR_calf = obs[23];

  obs_msg.vel_FL_hip = obs[24];   obs_msg.vel_FL_thigh = obs[25];   obs_msg.vel_FL_calf = obs[26];
  obs_msg.vel_FR_hip = obs[27];   obs_msg.vel_FR_thigh = obs[28];   obs_msg.vel_FR_calf = obs[29];
  obs_msg.vel_RL_hip = obs[30];   obs_msg.vel_RL_thigh = obs[31];   obs_msg.vel_RL_calf = obs[32];
  obs_msg.vel_RR_hip = obs[33];   obs_msg.vel_RR_thigh = obs[34];   obs_msg.vel_RR_calf = obs[35];

  obs_msg.act_FL_hip = obs[36];   obs_msg.act_FL_thigh = obs[37];   obs_msg.act_FL_calf = obs[38];
  obs_msg.act_FR_hip = obs[39];   obs_msg.act_FR_thigh = obs[40];   obs_msg.act_FR_calf = obs[41];
  obs_msg.act_RL_hip = obs[42];   obs_msg.act_RL_thigh = obs[43];   obs_msg.act_RL_calf = obs[44];
  obs_msg.act_RR_hip = obs[45];   obs_msg.act_RR_thigh = obs[46];   obs_msg.act_RR_calf = obs[47];

  pub_obs_.publish(obs_msg);
}


void Go1RLController::send_foot_pos(Go1CtrlStates &go1_ctrl_states) {
  unitree_legged_msgs::FootPos foot_pos_msg;

  Eigen::Matrix<float, 3, NUM_LEG> foot_pos_rel_float = go1_ctrl_states.foot_pos_rel.cast<float>();

  foot_pos_msg.FL_x = foot_pos_rel_float(0, 0);
  foot_pos_msg.FL_y = foot_pos_rel_float(1, 0);
  foot_pos_msg.FL_z = foot_pos_rel_float(2, 0);

  foot_pos_msg.FR_x = foot_pos_rel_float(0, 1);
  foot_pos_msg.FR_y = foot_pos_rel_float(1, 1);
  foot_pos_msg.FR_z = foot_pos_rel_float(2, 1);

  foot_pos_msg.RL_x = foot_pos_rel_float(0, 2);
  foot_pos_msg.RL_y = foot_pos_rel_float(1, 2);
  foot_pos_msg.RL_z = foot_pos_rel_float(2, 2);

  foot_pos_msg.RR_x = foot_pos_rel_float(0, 3);
  foot_pos_msg.RR_y = foot_pos_rel_float(1, 3);
  foot_pos_msg.RR_z = foot_pos_rel_float(2, 3);

  pub_foot_pos_rel_.publish(foot_pos_msg);

}

void Go1RLController::send_foot_force(Go1CtrlStates &go1_ctrl_states) {
  unitree_legged_msgs::FootForce foot_force_msg;

  Eigen::Vector4f foot_force_float = go1_ctrl_states.foot_force.cast<float>();

  foot_force_msg.FL = foot_force_float[0];
  foot_force_msg.FR = foot_force_float[1];
  foot_force_msg.RL = foot_force_float[2];
  foot_force_msg.RR = foot_force_float[3];

  pub_foot_force_.publish(foot_force_msg);

}