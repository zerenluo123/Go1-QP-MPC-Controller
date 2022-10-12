//
// Created by zixin on 11/1/21.
//

#include "GazeboA1ROS.hpp"

// constructor
GazeboA1ROS::GazeboA1ROS(ros::NodeHandle &_nh) {
    nh = _nh;

    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_calf_controller/command", 1);

    // debug estimation
    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/gazebo_go1/estimation_body_pose", 100);

    pub_euler_d = nh.advertise<geometry_msgs::PointStamped>("go1_debug/euler_d", 100);

    // ROS register callback, call backs directly modify variables in A1CtrlStates
    sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &GazeboA1ROS::gt_pose_callback, this);
    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &GazeboA1ROS::imu_callback, this);

    sub_joint_msg[0] = nh.subscribe("/go1_gazebo/FL_hip_controller/state", 2, &GazeboA1ROS::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/go1_gazebo/FL_thigh_controller/state", 2, &GazeboA1ROS::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/go1_gazebo/FL_calf_controller/state", 2, &GazeboA1ROS::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/go1_gazebo/FR_hip_controller/state", 2, &GazeboA1ROS::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/go1_gazebo/FR_thigh_controller/state", 2, &GazeboA1ROS::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/go1_gazebo/FR_calf_controller/state", 2, &GazeboA1ROS::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/go1_gazebo/RL_hip_controller/state", 2, &GazeboA1ROS::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/go1_gazebo/RL_thigh_controller/state", 2, &GazeboA1ROS::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/go1_gazebo/RL_calf_controller/state", 2, &GazeboA1ROS::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/go1_gazebo/RR_hip_controller/state", 2, &GazeboA1ROS::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/go1_gazebo/RR_thigh_controller/state", 2, &GazeboA1ROS::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/go1_gazebo/RR_calf_controller/state", 2, &GazeboA1ROS::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &GazeboA1ROS::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &GazeboA1ROS::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &GazeboA1ROS::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &GazeboA1ROS::RR_foot_contact_callback, this);

    sub_joy_msg = nh.subscribe("/joy", 1000, &GazeboA1ROS::joy_callback, this);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    _root_control = A1RobotControl(nh);
    go1_ctrl_states.reset();
    go1_ctrl_states.resetFromROSParam(nh);

    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_a1_body
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

    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);
}

bool GazeboA1ROS::update_foot_forces_grf(double dt) {
    go1_ctrl_states.foot_forces_grf = _root_control.compute_grf(go1_ctrl_states, dt);
    return true;
}

bool GazeboA1ROS::main_update(double dt) {
    if (joy_cmd_exit) {
        return false;
    }

    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into go1_ctrl_states
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

//    joy_cmd_body_height += joy_cmd_velz * dt;
//    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX + go1_ctrl_states.walking_surface_height) {
//        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX + go1_ctrl_states.walking_surface_height;
//    }
//    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN + go1_ctrl_states.walking_surface_height) {
//        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN + go1_ctrl_states.walking_surface_height;
//    }

    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;

    if (joy_cmd_ctrl_state_change_request) {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; //TODO: how to toggle more states?
        joy_cmd_ctrl_state_change_request = false; //erase this change request;
    }

    // root_lin_vel_d is in robot frame
    go1_ctrl_states.root_lin_vel_d[0] = joy_cmd_velx;
    go1_ctrl_states.root_lin_vel_d[1] = joy_cmd_vely;
    go1_ctrl_states.root_lin_vel_d[2] = joy_cmd_velz;

    // root_ang_vel_d is in robot frame
    go1_ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    go1_ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    go1_ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    go1_ctrl_states.root_euler_d[0] += joy_cmd_roll_rate * dt;
    go1_ctrl_states.root_euler_d[1] += joy_cmd_pitch_rate * dt;
    go1_ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    go1_ctrl_states.root_pos_d[2] = joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // walking mode, in this mode the robot should execute gait
        go1_ctrl_states.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        go1_ctrl_states.movement_mode = 0;
        go1_ctrl_states.root_pos_d.segment<2>(0) = go1_ctrl_states.root_pos.segment<2>(0);
        go1_ctrl_states.kp_linear(0) = go1_ctrl_states.kp_linear_lock_x;
        go1_ctrl_states.kp_linear(1) = go1_ctrl_states.kp_linear_lock_y;
    } else {
        go1_ctrl_states.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (go1_ctrl_states.movement_mode == 1) {
        if (go1_ctrl_states.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            go1_ctrl_states.root_pos_d.segment<2>(0) = go1_ctrl_states.root_pos.segment<2>(0);
            go1_ctrl_states.kp_linear.segment<2>(0).setZero();
        } else {
            go1_ctrl_states.kp_linear(0) = go1_ctrl_states.kp_linear_lock_x;
            go1_ctrl_states.kp_linear(1) = go1_ctrl_states.kp_linear_lock_y;
        }
    }

    _root_control.update_plan(go1_ctrl_states, dt);
    _root_control.generate_swing_legs_ctrl(go1_ctrl_states, dt);

    // state estimation
    if (!go1_estimate.is_inited()) {
        go1_estimate.init_state(go1_ctrl_states);
    } else {
        go1_estimate.update_estimation(go1_ctrl_states, dt);
    }

    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = go1_ctrl_states.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = go1_ctrl_states.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = go1_ctrl_states.estimated_root_pos(2);

    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = go1_ctrl_states.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = go1_ctrl_states.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = go1_ctrl_states.estimated_root_vel(2);

    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool GazeboA1ROS::send_cmd() {
    _root_control.compute_joint_torques(go1_ctrl_states);

    // send control cmd to robot via ros topic
    unitree_legged_msgs::LowCmd low_cmd;

    for (int i = 0; i < 12; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = go1_ctrl_states.joint_torques(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }

    // add a current joint state check if beyond the joint limit
//    std::cout << "All joint state: " << go1_ctrl_states.joint_pos << std::endl;
//    std::cout << "************ Is terminal state: " << isTerminalState(go1_ctrl_states.joint_pos) << std::endl;
//    if ( isTerminalState(go1_ctrl_states.joint_pos) ) {
//        return false;
//    }

    return true;
}

// callback functions
void GazeboA1ROS::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    // update
    go1_ctrl_states.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                  odom->pose.pose.orientation.x,
                                                  odom->pose.pose.orientation.y,
                                                  odom->pose.pose.orientation.z);                                              
    // go1_ctrl_states.root_pos << odom->pose.pose.position.x,
    //         odom->pose.pose.position.y,
    //         odom->pose.pose.position.z;
    // // make sure root_lin_vel is in world frame
    // go1_ctrl_states.root_lin_vel << odom->twist.twist.linear.x,
    //         odom->twist.twist.linear.y,
    //         odom->twist.twist.linear.z;

    // make sure root_ang_vel is in world frame
    // go1_ctrl_states.root_ang_vel << odom->twist.twist.angular.x,
    //         odom->twist.twist.angular.y,
    //         odom->twist.twist.angular.z;



    // calculate several useful variables
    // euler should be roll pitch yaw
    go1_ctrl_states.root_rot_mat = go1_ctrl_states.root_quat.toRotationMatrix();
    go1_ctrl_states.root_euler = Utils::quat_to_euler(go1_ctrl_states.root_quat);
    double yaw_angle = go1_ctrl_states.root_euler[2];

    go1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
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

        go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) = go1_ctrl_states.root_rot_mat * go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) = go1_ctrl_states.root_rot_mat * go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        go1_ctrl_states.foot_pos_world.block<3, 1>(0, i) = go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + go1_ctrl_states.root_pos;
        go1_ctrl_states.foot_vel_world.block<3, 1>(0, i) = go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + go1_ctrl_states.root_lin_vel;
    }
}

void GazeboA1ROS::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
    // go1_ctrl_states.root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(imu->orientation.w),
    //                                               quat_x.CalculateAverage(imu->orientation.x),
    //                                               quat_y.CalculateAverage(imu->orientation.y),
    //                                               quat_z.CalculateAverage(imu->orientation.z));
    go1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(imu->linear_acceleration.x),
            acc_y.CalculateAverage(imu->linear_acceleration.y),
            acc_z.CalculateAverage(imu->linear_acceleration.z)
    );
    go1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(imu->angular_velocity.x),
            gyro_y.CalculateAverage(imu->angular_velocity.y),
            gyro_z.CalculateAverage(imu->angular_velocity.z)
    );
    go1_ctrl_states.root_ang_vel = go1_ctrl_states.root_rot_mat * go1_ctrl_states.imu_ang_vel;
}

// FL
void GazeboA1ROS::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[0] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[0] = a1_joint_state.dq;
}

void GazeboA1ROS::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[1] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[1] = a1_joint_state.dq;
}

void GazeboA1ROS::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[2] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[2] = a1_joint_state.dq;
}

// FR
void GazeboA1ROS::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[3] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[3] = a1_joint_state.dq;
}

void GazeboA1ROS::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[4] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[4] = a1_joint_state.dq;
}

void GazeboA1ROS::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[5] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[5] = a1_joint_state.dq;
}

// RL
void GazeboA1ROS::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[6] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[6] = a1_joint_state.dq;
}

void GazeboA1ROS::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[7] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[7] = a1_joint_state.dq;
}

void GazeboA1ROS::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[8] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[8] = a1_joint_state.dq;
}

// RR
void GazeboA1ROS::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[9] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[9] = a1_joint_state.dq;
}

void GazeboA1ROS::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[10] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[10] = a1_joint_state.dq;
}

void GazeboA1ROS::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    go1_ctrl_states.joint_pos[11] = a1_joint_state.q;
    go1_ctrl_states.joint_vel[11] = a1_joint_state.dq;
}

// foot contact force
void GazeboA1ROS::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states.foot_force[0] = force.wrench.force.z;
}

void GazeboA1ROS::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states.foot_force[1] = force.wrench.force.z;
}

void GazeboA1ROS::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states.foot_force[2] = force.wrench.force.z;
}

void GazeboA1ROS::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    go1_ctrl_states.foot_force[3] = force.wrench.force.z;
}

void GazeboA1ROS::
joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

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
    // up-down button
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}

bool GazeboA1ROS::isTerminalState(Eigen::Matrix<double, NUM_DOF, 1> pos) {
    for (int i = 0; i < NUM_DOF; i++) {
        if (pos[i] <= posLimits[i % 3][0] || pos[i] >= posLimits[i % 3][1]) {
            return true;
        }
    }
    return false;
}
