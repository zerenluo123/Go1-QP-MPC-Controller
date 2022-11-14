//
// Created by zixin on 11/1/21.
//
// stl
#include "Go1RLHardwareController.hpp"

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "Go1Params.hpp"
#include "servo_stand_policy/HardwareServo.hpp"
#include "servo_stand_policy/HardwareServoSwitch.hpp"
#include "SwitchController.hpp"



int main(int argc, char **argv) {
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "Stand" << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  ros::init(argc, argv, "hardware_go1_rl_ctrl");
  ros::NodeHandle nh;

  double action_update_frequency, deployment_frequency;
  ros::param::get("action_update_frequency", action_update_frequency);
  ros::param::get("deployment_frequency", deployment_frequency);

  // change ros logger
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // make sure the ROS infra DO NOT use sim time, otherwise the controller cannot run with correct time steps
  std::string use_sim_time;
  if (ros::param::get("/use_sim_time", use_sim_time)) {
    if (use_sim_time != "false") {
      std::cout << "hardware must have real time in order to use this program!" << std::endl;
      return -1;
    }
  }

  // create go1 controllers
  std::unique_ptr<Go1RLHardwareController> go1_rl = std::make_unique<Go1RLHardwareController>(nh);
  std::unique_ptr<SwitchController> switch_ctrl = std::make_unique<SwitchController>(nh);
  std::unique_ptr<HardwareServo> servo = std::make_unique<HardwareServo>();
  std::unique_ptr<HardwareServoSwitch> servo_switch = std::make_unique<HardwareServoSwitch>();

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
    ros::Duration dt_solver_time(0);

    while (control_execute.load(std::memory_order_acquire) && ros::ok()) {

//      ros::Duration(action_update_frequency / 1000).sleep();

      // get t and dt
      now = ros::Time::now();
      dt = now - prev;
      prev = now;

      auto t1 = std::chrono::high_resolution_clock::now();

      bool running;
      switch_ctrl->updateMovementMode();
      if (switch_ctrl->movement_mode == 0) { // stand
        // servo controller
//        running = servo->UDPRecv();
        running = go1_rl->advance_servo();
      } else { // walk
//        running = servo_switch->UDPRecv();
        running = go1_rl->advance();
      }

      dt_solver_time = ros::Time::now() - now;

      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t2 - t1;
      if (switch_ctrl->movement_mode == 1) { // stand
        std::cout << "thread 1 solution is updated in " << ms_double.count() << "ms" << std::endl;
      }

      if (!running) {
        std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
        ros::shutdown();
        std::terminate();
        break;
      }

      if (dt_solver_time.toSec() < action_update_frequency / 1000) {
        ros::Duration( action_update_frequency / 1000 - dt_solver_time.toSec() ).sleep();
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
    ros::Duration dt_solver_time(0);

    while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
//      auto t3 = std::chrono::high_resolution_clock::now();

      // get t and dt
      now = ros::Time::now();
      dt = now - prev;
      prev = now;

      bool send_cmd_running = go1_rl->send_cmd();

      if (!send_cmd_running) {
        std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
        ros::shutdown();
        std::terminate();
        break;
      }

      dt_solver_time = ros::Time::now() - now;
      if (dt_solver_time.toSec() < deployment_frequency / 1000) {
        ros::Duration( deployment_frequency / 1000 - dt_solver_time.toSec() ).sleep();
      }
    }
  });

  ros::AsyncSpinner spinner(12);
  spinner.start();

  compute_action_thread.join();
  deploy_thread.join();

  return 0;

}


