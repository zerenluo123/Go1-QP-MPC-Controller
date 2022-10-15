//
// Created by shuoy on 11/7/21.
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

// control parameters
#include "Go1Params.hpp"
// TODO: RL control
// servo stand control
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

    // TODO: create RL controller
    // create servo stand controller
    std::unique_ptr<HardwareServo> servo = std::make_unique<HardwareServo>(UNITREE_LEGGED_SDK::LOWLEVEL);
    std::unique_ptr<HardwareServoSwitch> servo_switch = std::make_unique<HardwareServoSwitch>(UNITREE_LEGGED_SDK::LOWLEVEL);
    std::unique_ptr<SwitchController> switch_ctrl = std::make_unique<SwitchController>(nh);

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    // Thread 1: compute desired ground forces
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    std::thread compute_foot_forces_grf_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // auto t1 = std::chrono::high_resolution_clock::now();

//            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            // ros::Duration elapsed = now - start;
            // std::cout << "Thread 1 is updated in " << dt.toSec() << "s" << std::endl;

            bool running;
            switch_ctrl->updateMovementMode();
            if (switch_ctrl->movement_mode == 0) { // stand
              // servo controller
              running = servo->UDPRecv();
            } else {
              running = servo_switch->UDPRecv();
            }


            dt_solver_time = ros::Time::now() - now;

            // auto t2 = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            // std::cout << "foot force is solved in " << dt_solver_time.toSec() << "s" << std::endl;

            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            if (dt_solver_time.toSec() < ACTION_UPDATE_FREQUENCY / 1000) {
                ros::Duration( ACTION_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
            }
        }
    });

    // Thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands
     std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
               << std::endl;
    std::thread main_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // auto t3 = std::chrono::high_resolution_clock::now();


            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            bool send_cmd_running;
            switch_ctrl->updateMovementMode();
            if (switch_ctrl->movement_mode == 0) { // stand
              // servo controller
              send_cmd_running = servo->send_cmd();
            } else {
              send_cmd_running = servo_switch->send_cmd();
            }


            if (!send_cmd_running) {
                // std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            dt_solver_time = ros::Time::now() - now;
            if (dt_solver_time.toSec() < DEPLOYMENT_FREQUENCY / 1000) {
                ros::Duration( DEPLOYMENT_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
            }
        }
    });

    ros::AsyncSpinner spinner(12);
    spinner.start();

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
