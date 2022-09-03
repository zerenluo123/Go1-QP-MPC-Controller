//
// Created by zerenluo on 30.08.22.
//

#include "Go1RLController.hpp"


Go1RLController::Go1RLController(ros::NodeHandle &nh) {
  nh_ = nh;
  ros::param::get("package_dir", pkgDir_);
  ros::param::get("weights", ctrlWeights_);

  sub_joy_msg_ = nh.subscribe("/joy", 1000, &Go1RLController::joy_callback, this);

}


bool Go1RLController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLController::create] creating");

  //! load NN parameter
  loadNNparams();

  //! initialize observations & actions

  return true;

}

void Go1RLController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLController] Weights: " << ctrlWeights_ << "\033[0m");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLController::loadNNparams] load policy weights ");



}

bool Go1RLController::advance(double dt) {
  // test for loading
  auto input = Eigen::VectorXf::Ones(48);
  Eigen::VectorXf output;
  policy_.run(input, output);
  std::cout << output << std::endl;

  return true;

}


void Go1RLController::
joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
//  // left updown: change body height, not need now
//  joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

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

//bool Go1RLController::send_cmd() {
//  _root_control.compute_joint_torques(a1_ctrl_states);
//
//  // send control cmd to robot via ros topic
//  unitree_legged_msgs::LowCmd low_cmd;
//
//  for (int i = 0; i < 12; i++) {
//    low_cmd.motorCmd[i].mode = 0x0A;
//    low_cmd.motorCmd[i].q = 0;
//    low_cmd.motorCmd[i].dq = 0;
//    low_cmd.motorCmd[i].Kp = 0;
//    low_cmd.motorCmd[i].Kd = 0;
//    low_cmd.motorCmd[i].tau = a1_ctrl_states.joint_torques(i, 0);
//    pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
//  }
//
//  return true;
//}

