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

// servo stand control
#include "Lowlevel.hpp"
#include "HighLevel.hpp"

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
    std::unique_ptr<Lowlevel> servo = std::make_unique<Lowlevel>();
    std::unique_ptr<HighLevel> walk = std::make_unique<HighLevel>();

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
//             auto t1 = std::chrono::high_resolution_clock::now();

//            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;

//            walk->UDPRecv();
            bool running = walk->RobotControl();

            dt_solver_time = ros::Time::now() - now;



            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            if (dt_solver_time.toSec() < ACTION_UPDATE_FREQUENCY / 1000) {
                ros::Duration( ACTION_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
            }
//           auto t2 = std::chrono::high_resolution_clock::now();
//           std::chrono::duration<double, std::milli> ms_double = t2 - t1;
//           std::cout << "foot force is solved in " << dt_solver_time.toSec() << "s" << std::endl;
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

            walk->UDPRecv();
            bool send_cmd_running = walk->UDPSend();


            if (!send_cmd_running) {
                // std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            dt_solver_time = ros::Time::now() - now;
            if (dt_solver_time.toSec() < DEPLOYMENT_FREQUENCY / 1000) { // if this f is to high, will read many repetitive data
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
