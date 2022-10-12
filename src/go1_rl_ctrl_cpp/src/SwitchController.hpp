//
// Created by zerenluo on 12.10.22.
//

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

class SwitchController {
 public:
  SwitchController(ros::NodeHandle &nh) {
    sub_joy_msg_ = nh.subscribe("/joy", 1000, &SwitchController::joy_callback, this);

    joy_cmd_ctrl_state_ = 0;
    joy_cmd_ctrl_state_change_request_ = false;
    prev_joy_cmd_ctrl_state_ = 0;
    joy_cmd_exit_ = false;

  }

  void updateMovementMode() { // this is independent with the one in Go1 observation. Here is for changing the outside mode
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
      movement_mode = 1;
    } else if (joy_cmd_ctrl_state_ == 0 && prev_joy_cmd_ctrl_state_ == 1) {
      // leave walking mode
      // lock current position, should just happen for one instance
      movement_mode = 0;
    } else {
      movement_mode = 0;
    }


  } // updateMovementMode

  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    //A
    if (joy_msg->buttons[0] == 1) {
      joy_cmd_ctrl_state_change_request_ = true;
    }
  }

  int movement_mode;

 private:
  ros::Subscriber sub_joy_msg_;

  //  0 is standing, 1 is walking
  int joy_cmd_ctrl_state_ = 0;
  int prev_joy_cmd_ctrl_state_ = 0;
  bool joy_cmd_ctrl_state_change_request_ = false;
  bool joy_cmd_exit_ = false;



};

