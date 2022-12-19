//
// Created by zerenluo on 14.10.22.
//

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>

class GazeboServo
{
 public:
  GazeboServo(ros::NodeHandle &nh);
  bool state_pub();

  void paramInit();
  bool send_cmd();

  void imuCallback(const sensor_msgs::Imu & msg);

  void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
  void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
  void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);
  void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
  void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
  void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);
  void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
  void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
  void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);
  void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
  void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
  void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);

  void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
  void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
  void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
  void RLfootCallback(const geometry_msgs::WrenchStamped& msg);


 private:
  ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
  ros::Publisher servo_pub[12];
  ros::Publisher highState_pub;
  ros::Publisher lowState_pub; //for rviz visualization
  unitree_legged_msgs::LowCmd lowCmd;
  unitree_legged_msgs::LowState lowState;

  double motion_time;
  double duration = 1000;
  double target_pos[12];

  std::string robot_name;

};