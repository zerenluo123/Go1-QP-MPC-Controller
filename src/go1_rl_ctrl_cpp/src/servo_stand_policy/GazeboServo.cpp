//
// Created by zerenluo on 14.10.22.
//

#include "GazeboServo.hpp"

GazeboServo::GazeboServo(ros::NodeHandle &nh) {
  ros::param::get("robot_name", robot_name);

  // ROS publisher
  servo_pub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
  servo_pub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
  servo_pub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
  servo_pub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
  servo_pub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
  servo_pub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
  servo_pub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
  servo_pub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
  servo_pub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
  servo_pub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
  servo_pub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
  servo_pub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

  // ROS subscriber
  imu_sub = nh.subscribe("/trunk_imu", 1, &GazeboServo::imuCallback, this);
  footForce_sub[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &GazeboServo::FRfootCallback, this);
  footForce_sub[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &GazeboServo::FLfootCallback, this);
  footForce_sub[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &GazeboServo::RRfootCallback, this);
  footForce_sub[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &GazeboServo::RLfootCallback, this);
  servo_sub[0] = nh.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &GazeboServo::FRhipCallback, this);
  servo_sub[1] = nh.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &GazeboServo::FRthighCallback, this);
  servo_sub[2] = nh.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &GazeboServo::FRcalfCallback, this);
  servo_sub[3] = nh.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &GazeboServo::FLhipCallback, this);
  servo_sub[4] = nh.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &GazeboServo::FLthighCallback, this);
  servo_sub[5] = nh.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &GazeboServo::FLcalfCallback, this);
  servo_sub[6] = nh.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &GazeboServo::RRhipCallback, this);
  servo_sub[7] = nh.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &GazeboServo::RRthighCallback, this);
  servo_sub[8] = nh.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &GazeboServo::RRcalfCallback, this);
  servo_sub[9] = nh.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &GazeboServo::RLhipCallback, this);
  servo_sub[10] = nh.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &GazeboServo::RLthighCallback, this);
  servo_sub[11] = nh.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &GazeboServo::RLcalfCallback, this);

  // low state publisher
  lowState_pub = nh.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);

  motion_time = 0.;

  // motor parameter init
//  paramInit();

}

void GazeboServo::paramInit()
{
  for(int i=0; i<2; i++){
    lowCmd.motorCmd[i*3+0].mode = 0x0A;
    lowCmd.motorCmd[i*3+0].Kp = 20;
    lowCmd.motorCmd[i*3+0].dq = 0;
    lowCmd.motorCmd[i*3+0].Kd = 5;
    lowCmd.motorCmd[i*3+0].tau = 0;
    lowCmd.motorCmd[i*3+1].mode = 0x0A;
    lowCmd.motorCmd[i*3+1].Kp = 30;
    lowCmd.motorCmd[i*3+1].dq = 0;
    lowCmd.motorCmd[i*3+1].Kd = 8;
    lowCmd.motorCmd[i*3+1].tau = 0;
    lowCmd.motorCmd[i*3+2].mode = 0x0A;
    lowCmd.motorCmd[i*3+2].Kp = 60;
    lowCmd.motorCmd[i*3+2].dq = 0;
    lowCmd.motorCmd[i*3+2].Kd = 12;
    lowCmd.motorCmd[i*3+2].tau = 0;
  }

  for(int i=2; i<4; i++){
    lowCmd.motorCmd[i*3+0].mode = 0x0A;
    lowCmd.motorCmd[i*3+0].Kp = 20;
    lowCmd.motorCmd[i*3+0].dq = 0;
    lowCmd.motorCmd[i*3+0].Kd = 5;
    lowCmd.motorCmd[i*3+0].tau = 0;
    lowCmd.motorCmd[i*3+1].mode = 0x0A;
    lowCmd.motorCmd[i*3+1].Kp = 80;
    lowCmd.motorCmd[i*3+1].dq = 0;
    lowCmd.motorCmd[i*3+1].Kd = 8;
    lowCmd.motorCmd[i*3+1].tau = 0;
    lowCmd.motorCmd[i*3+2].mode = 0x0A;
    lowCmd.motorCmd[i*3+2].Kp = 140;
    lowCmd.motorCmd[i*3+2].dq = 0;
    lowCmd.motorCmd[i*3+2].Kd = 12;
    lowCmd.motorCmd[i*3+2].tau = 0;
  }
}

bool GazeboServo::state_pub() {
  double targetPos[12] = {0.1, 0.6, -1.3, -0.1, 0.6, -1.3,
                    0.1, 0.6, -1.3, -0.1, 0.6, -1.3};

  paramInit();

  motion_time += 1.0; // duration count
  double pos[12] ,lastPos[12], percent;
  for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
  percent = (double)motion_time/duration;
  for(int j=0; j<12; j++){
    target_pos[j] = lastPos[j]*(1-percent) + targetPos[j]*percent;
  }

  return true;
}

bool GazeboServo::send_cmd() {
  for(int j=0; j<12; j++){
    lowCmd.motorCmd[j].q = target_pos[j];
    servo_pub[j].publish(lowCmd.motorCmd[j]);
  }

  return true;

}

void GazeboServo::imuCallback(const sensor_msgs::Imu & msg)
{
  lowState.imu.quaternion[0] = msg.orientation.w;
  lowState.imu.quaternion[1] = msg.orientation.x;
  lowState.imu.quaternion[2] = msg.orientation.y;
  lowState.imu.quaternion[3] = msg.orientation.z;

  lowState.imu.gyroscope[0] = msg.angular_velocity.x;
  lowState.imu.gyroscope[1] = msg.angular_velocity.y;
  lowState.imu.gyroscope[2] = msg.angular_velocity.z;

  lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
  lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
  lowState.imu.accelerometer[2] = msg.linear_acceleration.z;

}

void GazeboServo::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[0].mode = msg.mode;
  lowState.motorState[0].q = msg.q;
  lowState.motorState[0].dq = msg.dq;
  lowState.motorState[0].tauEst = msg.tauEst;
}

void GazeboServo::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[1].mode = msg.mode;
  lowState.motorState[1].q = msg.q;
  lowState.motorState[1].dq = msg.dq;
  lowState.motorState[1].tauEst = msg.tauEst;
}

void GazeboServo::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[2].mode = msg.mode;
  lowState.motorState[2].q = msg.q;
  lowState.motorState[2].dq = msg.dq;
  lowState.motorState[2].tauEst = msg.tauEst;
}

void GazeboServo::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[3].mode = msg.mode;
  lowState.motorState[3].q = msg.q;
  lowState.motorState[3].dq = msg.dq;
  lowState.motorState[3].tauEst = msg.tauEst;
}

void GazeboServo::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[4].mode = msg.mode;
  lowState.motorState[4].q = msg.q;
  lowState.motorState[4].dq = msg.dq;
  lowState.motorState[4].tauEst = msg.tauEst;
}

void GazeboServo::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[5].mode = msg.mode;
  lowState.motorState[5].q = msg.q;
  lowState.motorState[5].dq = msg.dq;
  lowState.motorState[5].tauEst = msg.tauEst;
}

void GazeboServo::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[6].mode = msg.mode;
  lowState.motorState[6].q = msg.q;
  lowState.motorState[6].dq = msg.dq;
  lowState.motorState[6].tauEst = msg.tauEst;
}

void GazeboServo::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[7].mode = msg.mode;
  lowState.motorState[7].q = msg.q;
  lowState.motorState[7].dq = msg.dq;
  lowState.motorState[7].tauEst = msg.tauEst;
}

void GazeboServo::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[8].mode = msg.mode;
  lowState.motorState[8].q = msg.q;
  lowState.motorState[8].dq = msg.dq;
  lowState.motorState[8].tauEst = msg.tauEst;
}

void GazeboServo::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[9].mode = msg.mode;
  lowState.motorState[9].q = msg.q;
  lowState.motorState[9].dq = msg.dq;
  lowState.motorState[9].tauEst = msg.tauEst;
}

void GazeboServo::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[10].mode = msg.mode;
  lowState.motorState[10].q = msg.q;
  lowState.motorState[10].dq = msg.dq;
  lowState.motorState[10].tauEst = msg.tauEst;
}

void GazeboServo::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
  lowState.motorState[11].mode = msg.mode;
  lowState.motorState[11].q = msg.q;
  lowState.motorState[11].dq = msg.dq;
  lowState.motorState[11].tauEst = msg.tauEst;
}

void GazeboServo::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
  lowState.eeForce[0].x = msg.wrench.force.x;
  lowState.eeForce[0].y = msg.wrench.force.y;
  lowState.eeForce[0].z = msg.wrench.force.z;
  lowState.footForce[0] = msg.wrench.force.z;
}

void GazeboServo::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
  lowState.eeForce[1].x = msg.wrench.force.x;
  lowState.eeForce[1].y = msg.wrench.force.y;
  lowState.eeForce[1].z = msg.wrench.force.z;
  lowState.footForce[1] = msg.wrench.force.z;
}

void GazeboServo::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
  lowState.eeForce[2].x = msg.wrench.force.x;
  lowState.eeForce[2].y = msg.wrench.force.y;
  lowState.eeForce[2].z = msg.wrench.force.z;
  lowState.footForce[2] = msg.wrench.force.z;
}

void GazeboServo::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
  lowState.eeForce[3].x = msg.wrench.force.x;
  lowState.eeForce[3].y = msg.wrench.force.y;
  lowState.eeForce[3].z = msg.wrench.force.z;
  lowState.footForce[3] = msg.wrench.force.z;
}
