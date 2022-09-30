//
// Created by zerenluo on 29.09.22.
//

#include "Go1RLHardwareController.hpp"

// constructor
Go1RLHardwareController::Go1RLHardwareController(ros::NodeHandle &nh)
    : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007) {
  nh_ = nh;
  ros::param::get("package_dir", pkgDir_);
  ros::param::get("weights", ctrlWeights_);
  ros::param::get("stand_weights", standCtrlWeights_);

  udp.InitCmdData(cmd);
  udp_init_send();

  go1_ctrl_states.reset();
  go1_ctrl_states.resetFromROSParam(nh_);

  //init swap order, very important
  swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
  swap_foot_indices << 1, 0, 3, 2;

  // action and torque init
  actionDouble_.setZero(12);  prevActionDouble_.setZero(12);  action_.setZero(12);
  torques_.setZero(12);
  // observation
  go1Obs_ = std::make_unique<Go1HardwareObservation>(nh);

  // start hardware reading thread after everything initialized TODO: change this into hardware observation
  thread_ = std::thread(&Go1RLHardwareController::receive_low_state, this);

}


bool Go1RLHardwareController::create(double dt) {

  // Load parameters
  ROS_INFO_STREAM("[Go1RLHardwareController::create] creating");

  //! load NN parameter
  loadNNparams();

  return true;
}

void Go1RLHardwareController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLHardwareController] Weights: " << standCtrlWeights_ << "\033[0m");
  ROS_INFO_STREAM("\033[1;33m[Go1RLHardwareController] Weights: " << ctrlWeights_ << "\033[0m");

  standPolicy_.load(pkgDir_ + "/resource/" + standCtrlWeights_);
  ROS_INFO_STREAM("[Go1RLHardwareController::loadNNparams] load stand policy weights ");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLHardwareController::loadNNparams] load policy weights ");

}


bool Go1RLHardwareController::send_cmd() {
  // send control cmd to robot via unitree hardware interface
  // notice go1_ctrl_states.joint_torques uses order FL, FR, RL, RR
  // notice cmd uses order FR, FL, RR, RL
  cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
  for (int i = 0; i < NUM_DOF; i++) {
    cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
    cmd.motorCmd[i].Kp = 0;
    cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
    cmd.motorCmd[i].Kd = 0;
    int swap_i = swap_joint_indices(i);
    cmd.motorCmd[i].tau = go1_ctrl_states.joint_torques(swap_i);

    std::cout << cmd.motorCmd[i].tau << std::endl;


  }

  std::cout << "************** finish print tau ***************" << std::endl;

  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, go1_ctrl_states.power_level);
  safe.PositionProtect(cmd, state, -0.2);
  udp.SetSend(cmd);
  udp.Send();

  return true;
}


void Go1RLHardwareController::udp_init_send() {
  cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
  for (int i = 0; i < NUM_DOF; i++) {
    cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // 禁止位置环
    cmd.motorCmd[i].Kp = 0;
    cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;        // 禁止速度环
    cmd.motorCmd[i].Kd = 0;
    cmd.motorCmd[i].tau = 0;
  }
  safe.PositionLimit(cmd);
  safe.PositionProtect(cmd, state, -0.2);
  udp.SetSend(cmd);
  udp.Send();
}