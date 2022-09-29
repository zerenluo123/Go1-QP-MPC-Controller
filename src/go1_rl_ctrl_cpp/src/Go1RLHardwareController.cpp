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

  // ROS publisher
  pub_joint_cmd = nh_.advertise<sensor_msgs::JointState>("/hardware_go1/joint_torque_cmd", 100);

  // debug joint angle and foot force
  pub_joint_angle = nh_.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 100);

  // imu data
  pub_imu = nh_.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 100);

  sub_joy_msg = nh_.subscribe("/joy", 1000, &Go1RLHardwareController::joy_callback, this);

  udp.InitCmdData(cmd);
  udp_init_send();

  joy_cmd_ctrl_state = 0;
  joy_cmd_ctrl_state_change_request = false;
  prev_joy_cmd_ctrl_state = 0;
  joy_cmd_exit = false;

  go1_ctrl_states.reset();
  go1_ctrl_states.resetFromROSParam(nh_);

  //init swap order, very important
  swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
  swap_foot_indices << 1, 0, 3, 2;

  // start hardware reading thread after everything initialized
  thread_ = std::thread(&Go1RLHardwareController::receive_low_state, this);

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

void Go1RLHardwareController::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
//  // left updown
//  joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

  //A
  if (joy_msg->buttons[0] == 1) {
    joy_cmd_ctrl_state_change_request = true;
  }

  // right updown
  joy_cmd_velx = joy_msg->axes[4] * JOY_CMD_VELX_MAX;
  // right horiz
  joy_cmd_vely = joy_msg->axes[3] * JOY_CMD_VELY_MAX;
  // left horiz
  joy_cmd_yaw_rate  = joy_msg->axes[0]*JOY_CMD_YAW_MAX;

  // lb
  if (joy_msg->buttons[4] == 1) {
    std::cout << "You have pressed the exit button!!!!" << std::endl;
    joy_cmd_exit = true;
  }
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