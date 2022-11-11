//
// Created by zerenluo on 14.10.22.
//

// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

#include <Eigen/Dense>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <vector>

#include <ros/ros.h>


#define ACTION_UPDATE_FREQUENCY 10.0 // ms
#define DEPLOYMENT_FREQUENCY 5.0 // ms
#define HARDWARE_FEEDBACK_FREQUENCY 10.0  // ms

#define N_JOINTS 12
#define SIGNAL_LENGTH 800

class HighLevel
{
 public:
  HighLevel();

//  ~HighLevel() {
//    destruct = true;
//    thread_.join();
//  }

  void UDPRecv();
  bool UDPSend();
  bool RobotControl();
  void updateMovementScheme(int i);
  void saveSignalAsFile();
  void initMotionScheme();


  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::HighCmd cmd = {0};
  UNITREE_LEGGED_SDK::HighState state = {0};
  int motiontime = 0;

  int MOVEMENT_DURATION = 100;
  int PAUSE_DURATION = 0;

 private:
  float cmdVelX_, cmdVelY_, cmdVelMag_, cmdVelAng_; // change the direction/magnitude of the velocity
  vector<float> qSignal_;

  // the list of everything
  float velDir_, velMag_, footHeight_, bodyHeight_;
  std::vector<float> motionParam_;
  Eigen::MatrixXf motionParamMat_;

  Eigen::MatrixXf qSignalMat_;

  void repeatCheck();

  // a1 hardware reading thread
  std::thread thread_;
  bool destruct = false;


};