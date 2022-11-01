//
// Created by zixin on 11/1/21.
//
// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

//#include "torch_eigen/TorchEigen.hpp"
#include "Go1RLController.hpp"
#include "SwitchController.hpp"
#include "servo_stand_policy/GazeboServo.hpp"


int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_go1_rl_ctrl");
  ros::NodeHandle nh;

  double action_update_frequency, deployment_frequency;
  ros::param::get("action_update_frequency", action_update_frequency);
  ros::param::get("deployment_frequency", deployment_frequency);

  // change ros logger
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // make sure the ROS infra using sim time, otherwise the controller cannot run with correct time steps
  bool use_sim_time;
  if (ros::param::get("use_sim_time", use_sim_time)) {
    if (!use_sim_time) {
      ROS_WARN_STREAM(" ROS must set use_sim_time in order to use this program! ");
      return -1;
    }
  }

  // create go1 controllers
  std::unique_ptr<Go1RLController> go1_rl = std::make_unique<Go1RLController>(nh);
  std::unique_ptr<SwitchController> switch_ctrl = std::make_unique<SwitchController>(nh);
  std::unique_ptr<GazeboServo> servo = std::make_unique<GazeboServo>(nh);


  std::atomic<bool> control_execute{};
  control_execute.store(true, std::memory_order_release);

  // Thread 1: compute command, forward the NN policy network
  std::cout << "Enter thread 1: compute desired action command" << std::endl;
  std::thread compute_action_thread([&]() {
    // prepare variables to monitor time and control the while loop
    ros::Time start = ros::Time::now();
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();  // bool res = app.exec();
    ros::Duration dt(0);

    while (control_execute.load(std::memory_order_acquire) && ros::ok()) {

      ros::Duration(action_update_frequency / 1000).sleep();

      // get t and dt
      now = ros::Time::now();
      dt = now - prev;
      prev = now;

      auto t1 = std::chrono::high_resolution_clock::now();

      bool running;
      switch_ctrl->updateMovementMode();
      if (switch_ctrl->movement_mode == 0) { // stand
        // compute desired ground forces
//        running = go1->update_foot_forces_grf(dt.toSec());
        running = servo->state_pub();
      } else { // walk
        // compute actions
        running = go1_rl->advance(dt.toSec());
      }

      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t2 - t1;
//      if (switch_ctrl->movement_mode == 1) { // stand
//        std::cout << "Controller solution is updated in " << ms_double.count() << "ms" << std::endl;
//      }

      if (!running) {
        std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
        ros::shutdown();
        std::terminate();
        break;
      }
    }

  });

  // Thread 2: depolyment on the real system; send commands
  std::cout << "Enter thread 2: Deployment on the real robot, sending command" << std::endl;
  std::thread deploy_thread([&]() {
    // prepare variables to monitor time and control the while loop
    ros::Time start = ros::Time::now();
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();  // bool res = app.exec();
    ros::Duration dt(0);

    while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
      auto t3 = std::chrono::high_resolution_clock::now();

      ros::Duration(deployment_frequency / 1000).sleep();

      // get t and dt
      now = ros::Time::now();
      dt = now - prev;
      prev = now;

      bool send_cmd_running;
      switch_ctrl->updateMovementMode();
      if (switch_ctrl->movement_mode == 0) { // stand
//        bool main_update_running = go1->main_update(dt.toSec());
//        send_cmd_running = go1->send_cmd();
        send_cmd_running = servo->send_cmd();
      } else { // walk
        send_cmd_running = go1_rl->send_cmd();
      }

      auto t4 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t4 - t3;
//      std::cout << "Thread 2 is updated in " << ms_double.count() << "ms" << std::endl;

      if (!send_cmd_running) {
        std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
        ros::shutdown();
        std::terminate();
        break;
      }

    }
  });

  ros::AsyncSpinner spinner(12);
  spinner.start();

  compute_action_thread.join();
  deploy_thread.join();

  return 0;

}


