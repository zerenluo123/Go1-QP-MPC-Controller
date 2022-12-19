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
  ros::param::get("stiffness", stiffness_);
  ros::param::get("damping", damping_);

  // ROS publisher
  // debug joint angle and foot force
  pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 100);

  // imu data
  pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 100);

  joint_foot_msg.name = {"FL0", "FL1", "FL2",
                         "FR0", "FR1", "FR2",
                         "RL0", "RL1", "RL2",
                         "RR0", "RR1", "RR2",
                         "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
  joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
  joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

  sub_joy_msg = nh.subscribe("/joy", 1000, &Go1RLHardwareController::joy_callback, this);

  udp.InitCmdData(cmd);
  udp_init_send();

  joy_cmd_ctrl_state = 0;
  joy_cmd_ctrl_state_change_request = false;
  prev_joy_cmd_ctrl_state = 0;
  joy_cmd_exit = false;

  go1_ctrl_states.reset();
  go1_ctrl_states.resetFromROSParam(nh);

  // init leg kinematics
  // set leg kinematics related parameters
  // body_to_go1_body
  p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
  R_br = Eigen::Matrix3d::Identity();
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
  footForceOffset_.setZero(4);
  footForceOffset_ << 3.0, 22.0, 92.0, 72.0;

  //init swap order, very important
  swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
  swap_foot_indices << 1, 0, 3, 2;

  // go1 hardware foot force filter reset
  foot_force_filters.setZero();
  foot_force_filters_idx.setZero();
  foot_force_filters_sum.setZero();

  // observation init
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

  // action and pose init
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

  servo_motion_time_ = 0.;

  //  // observation(the hardware reading thread is inited in here)
//  go1Obs_ = std::make_unique<Go1HardwareObservation>(nh);

  // Load parameters
  ROS_INFO_STREAM("[Go1RLHardwareController::create] creating");
  loadNNparams();

  // start hardware reading thread after everything initialized
  thread_ = std::thread(&Go1RLHardwareController::receive_low_state, this);

}

void Go1RLHardwareController::loadNNparams() {
  ROS_INFO_STREAM("\033[1;33m[Go1RLHardwareController] Weights: " << standCtrlWeights_ << "\033[0m");
  ROS_INFO_STREAM("\033[1;33m[Go1RLHardwareController] Weights: " << ctrlWeights_ << "\033[0m");

  standPolicy_.load(pkgDir_ + "/resource/" + standCtrlWeights_);
  ROS_INFO_STREAM("[Go1RLHardwareController::loadNNparams] load stand policy weights ");

  policy_.load(pkgDir_ + "/resource/" + ctrlWeights_);
  ROS_INFO_STREAM("[Go1RLHardwareController::loadNNparams] load policy weights ");

}

bool Go1RLHardwareController::advance() {
  if (joy_cmd_exit) {
    std::cout << "exit " << std::endl;
    return false;
  }

  pGains_ << 25., 45., 45.,
      25., 45., 45.,
      25., 45., 45.,
      25., 45., 45.;
  dGains_ << 4., 4., 4.,
      4., 4., 4.,
      4., 4., 4.,
      4., 4., 4.;

  // updata obs except for actions.
  // different from the gazebo, the hardware receive state measurement in controller, need to pass the control state to update observation
  updateObservation();
  Eigen::VectorXd proprioObs = obScaled_; // dim=36

  // put in action
  Eigen::VectorXf obs(proprioObs.size() + 12); // full observation; dim=48
  obs << proprioObs.cast<float>(), prevActionDouble_.cast<float>();

  policy_.run(obs, action_);
  actionDouble_ = action_.cast<double>();

  // add clip to the action(should be of little use)
  actionDouble_ = actionDouble_.cwiseMin(clipAction_).cwiseMax(-clipAction_);
  prevActionDouble_ = actionDouble_;

  // compute joint target pose (PD controller)
  Eigen::VectorXd actionScaled = actionDouble_ * actionScale_;
  targetPoses_ = actionScaled + go1_ctrl_states.default_joint_pos;
  // add clip to target poses
  targetPoses_ = targetPoses_.cwiseMin(clipPoseUpper_).cwiseMax(clipPoseLower_);


//  // *********** debug **********
//  send_obs(obs);

  return true;

}


bool Go1RLHardwareController::advance_servo() {
  double targetPos[12] = {0.1, 0.6, -1.3, -0.1, 0.6, -1.3,
                          0.1, 0.6, -1.3, -0.1, 0.6, -1.3};

  pGains_ << 10., 40., 50.,
      10., 40., 50.,
      10., 40., 50.,
      10., 40., 50.;
  dGains_ << 3., 4., 2.,
      3., 4., 2.,
      3., 4., 2.,
      3., 4., 2.;

  servo_motion_time_ += 1.0; // duration count
  double pos[12] ,lastPos[12], percent;
  for(int j=0; j<12; j++) lastPos[j] = go1_ctrl_states.joint_pos[j];
  percent = (double)servo_motion_time_/5000;
  for(int j=0; j<12; j++){
    targetPoses_[j] = lastPos[j]*(1-percent) + targetPos[j]*percent;
  }

  return true;

}


bool Go1RLHardwareController::send_cmd() {
  // send control cmd to robot via unitree hardware interface
  // notice go1_ctrl_states.joint_torques uses order FL, FR, RL, RR
  // notice cmd uses order FR, FL, RR, RL
  cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
  for (int i = 0; i < NUM_DOF; i++) {
    cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    cmd.motorCmd[i].tau = 0; // shut down torque control
    cmd.motorCmd[i].dq = 0; // shut down velocity control
    int swap_i = swap_joint_indices(i);
    cmd.motorCmd[i].q = targetPoses_(swap_i); // use target pose calculated in advance function
    cmd.motorCmd[i].Kp = pGains_(swap_i);
    cmd.motorCmd[i].Kd = dGains_(swap_i);
  }

  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, go1_ctrl_states.power_level);
//  safe.PositionProtect(cmd, state, -0.2);
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
//  safe.PositionProtect(cmd, state, -0.2);
  udp.SetSend(cmd);
  udp.Send();
}

// ***********************************************************************
// ********************** transfer from observation **********************
// ***********************************************************************
void Go1RLHardwareController::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) { // This function applies for both gazebo and hardware
//  // left updown: change body height, not need now
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
  joy_cmd_yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;

  // lb
  if (joy_msg->buttons[4] == 1) {
    std::cout << "You have pressed the exit button!!!!" << std::endl;
    joy_cmd_exit = true;
  }
}


void Go1RLHardwareController::updateObservation() {
  // TODO: get all observation from measurement and concat
  // assign observation vector
  obDouble_.segment(0, 3) = go1_ctrl_states.root_rot_mat_z.transpose() * go1_ctrl_states.root_lin_vel; // 1. base linear velocity(robot frame)
  obDouble_.segment(3, 3) = go1_ctrl_states.imu_ang_vel; // 2. base angular velocity(robot frame, [roll, pitch, yaw])
  obDouble_.segment(6, 3) = go1_ctrl_states.root_rot_mat.transpose() * (-Eigen::Vector3d::UnitZ()); // 3. projected gravity(projected z unit vector)
  obDouble_.segment(9, 3) << joy_cmd_velx, joy_cmd_vely, joy_cmd_yaw_rate; // 4. command([cmd_velx, cmd_vely, cmd_ang_vel_yaw])
  obDouble_.segment(12, 12) = go1_ctrl_states.joint_pos - go1_ctrl_states.default_joint_pos; // 5. (joint_pos - default_joint_pos)
  obDouble_.segment(24, 12) = go1_ctrl_states.joint_vel; // 6. joint_vel
//    obDouble_.segment(36, 12) = go1_ctrl_states.joint_actions; // 7. actions(clipped NN outputs)

  // scale the observation
  for (int i = 0; i < obDouble_.size(); i++) {
    obScaled_[i] = obDouble_[i] * scaleFactor_[i];
  }

  // clip the observation
  obScaled_ = obScaled_.cwiseMin(clipObs_).cwiseMax(-clipObs_);

  updateMovementMode();

}

void Go1RLHardwareController::updateMovementMode() { // keep this: the movement mode in ctrl state is used in the EKF.
  // update joy cmd
  prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;

  if (joy_cmd_ctrl_state_change_request) {
    // toggle joy_cmd_ctrl_state
    joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
    joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; //TODO: how to toggle more states?
    joy_cmd_ctrl_state_change_request = false; //erase this change request;
  }

  // determine movement mode
  if (joy_cmd_ctrl_state == 1) {
    // walking mode, in this mode the robot should execute gait
    go1_ctrl_states.movement_mode = 1;
  } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
    // leave walking mode
    go1_ctrl_states.movement_mode = 0;
  } else {
    go1_ctrl_states.movement_mode = 0;
  }

} // updateMovementMode



void Go1RLHardwareController::receive_low_state() {
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

    // fill data to go1_ctrl_states, notice the order in state is FR, FL, RR, RL
    // fill data to go1_ctrl_states, notice the order in go1_ctrl_states is FL, FR, RL, RR
    /* TODO: fill data */

    go1_ctrl_states.root_quat = Eigen::Quaterniond(state.imu.quaternion[0],
                                                  state.imu.quaternion[1],
                                                  state.imu.quaternion[2],
                                                  state.imu.quaternion[3]);
    go1_ctrl_states.root_rot_mat = go1_ctrl_states.root_quat.toRotationMatrix();
    go1_ctrl_states.root_euler = Utils::quat_to_euler(go1_ctrl_states.root_quat);
    double yaw_angle = go1_ctrl_states.root_euler[2];

    go1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
    // go1_ctrl_states.root_pos     | do not fill
    // go1_ctrl_states.root_lin_vel | do not fill

    go1_ctrl_states.imu_acc = Eigen::Vector3d(state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
    go1_ctrl_states.imu_ang_vel = Eigen::Vector3d(state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
    go1_ctrl_states.root_ang_vel = go1_ctrl_states.root_rot_mat * go1_ctrl_states.imu_ang_vel;

    // joint states
    // Get dt (in seconds)
    now = ros::Time::now();
    dt = now - prev;
    prev = now;
    double dt_s = dt.toSec();

    for (int i = 0; i < NUM_DOF; ++i) {
      int swap_i = swap_joint_indices(i);
      go1_ctrl_states.joint_vel[i] = state.motorState[swap_i].dq;
      // go1_ctrl_states.joint_vel[i] = (state.motorState[swap_i].q - go1_ctrl_states.joint_pos[i])/dt_s;
      go1_ctrl_states.joint_pos[i] = state.motorState[swap_i].q;
    }

    // foot force, add a filter here
    for (int i = 0; i < NUM_LEG; ++i) {
      int swap_i = swap_foot_indices(i);
      double value = static_cast<double>(state.footForce[swap_i]);

      foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_filters_idx[i]);
      foot_force_filters(i, foot_force_filters_idx[i]) = value;
      foot_force_filters_sum[i] += value;
      foot_force_filters_idx[i]++;
      foot_force_filters_idx[i] %= FOOT_FILTER_WINDOW_SIZE;

      go1_ctrl_states.foot_force[i] = foot_force_filters_sum[i] / static_cast<double>(FOOT_FILTER_WINDOW_SIZE) - footForceOffset_[i];
    }

    // publish joint angle and foot force
    for (int i = 0; i < NUM_DOF; ++i) {
      joint_foot_msg.position[i] = go1_ctrl_states.joint_pos[i];
      joint_foot_msg.velocity[i] = go1_ctrl_states.joint_vel[i];
    }
    for (int i = 0; i < NUM_LEG; ++i) {
      // publish plan contacts to help state estimation
      joint_foot_msg.velocity[NUM_DOF + i] = go1_ctrl_states.plan_contacts[i];
      joint_foot_msg.effort[NUM_DOF + i] = go1_ctrl_states.foot_force[i];
    }
    joint_foot_msg.header.stamp = ros::Time::now();
    pub_joint_angle.publish(joint_foot_msg);

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.angular_velocity.x = state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = state.imu.gyroscope[1];
    imu_msg.angular_velocity.z = state.imu.gyroscope[2];

    imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state.imu.accelerometer[2];
    pub_imu.publish(imu_msg);


//        std::cout << "go1_ctrl_states.foot_force.transpose()" << std::endl;
//        std::cout << go1_ctrl_states.foot_force.transpose() << std::endl;

    // TODO: shall we call estimator update here, be careful the runtime should smaller than the HARDWARE_FEEDBACK_FREQUENCY

    // state estimation
    auto t1 = ros::Time::now();
    if (!go1_estimate.is_inited()) {
      go1_estimate.init_state(go1_ctrl_states);
    } else {
      go1_estimate.update_estimation(go1_ctrl_states, dt_s);
    }
    auto t2 = ros::Time::now();
    ros::Duration run_dt = t2 - t1;

    // FL, FR, RL, RR
    // use estimation pos and vel to get foot pos and foot vel in world frame
    for (int i = 0; i < NUM_LEG; ++i) {
      go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = go1_kin.fk(
          go1_ctrl_states.joint_pos.segment<3>(3 * i),
          rho_opt_list[i], rho_fix_list[i]);
      go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = go1_kin.jac(
          go1_ctrl_states.joint_pos.segment<3>(3 * i),
          rho_opt_list[i], rho_fix_list[i]);
      Eigen::Matrix3d tmp_mtx = go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
      Eigen::Vector3d tmp_vec = go1_ctrl_states.joint_vel.segment<3>(3 * i);
      go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

      go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) =
          go1_ctrl_states.root_rot_mat * go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
      go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) =
          go1_ctrl_states.root_rot_mat * go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

      // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
      // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
      // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
      go1_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
          go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + go1_ctrl_states.root_pos;
      go1_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
          go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + go1_ctrl_states.root_lin_vel;
    }
    double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
    // sleep for interval_ms
    double interval_time = interval_ms / 1000.0;
    if (interval_time > run_dt.toSec()) {
      ros::Duration(interval_time - run_dt.toSec()).sleep();
    }
  };
}