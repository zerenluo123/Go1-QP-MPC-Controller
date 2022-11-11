//
// Created by zerenluo on 02.11.22.
//

#include "HighLevel.hpp"


HighLevel::HighLevel():
    safe(UNITREE_LEGGED_SDK::LeggedType::Go1),
    udp(UNITREE_LEGGED_SDK::HIGHLEVEL, 8090, "192.168.123.161", 8082){
  udp.InitCmdData(cmd);
  cmdVelX_ = 0.; cmdVelY_ = 0.; cmdVelAng_ = 0.; cmdVelMag_ = 0.;

  initMotionScheme();

//  // start hardware reading thread after everything initialized
//  thread_ = std::thread(&HighLevel::UDPRecv, this);
}

//void HighLevel::UDPRecv()
//{
//  while (destruct == false) {
//
//    udp.Recv();
//    udp.GetRecv(state);
//    for (int i = 0; i < N_JOINTS; i++) { // first try with front legs
//      qSignal_.push_back(state.motorState[i].q);
//    }
//
//    ros::Duration(HARDWARE_FEEDBACK_FREQUENCY / 1000.0).sleep();
//  }
//
//}

void HighLevel::UDPRecv()
{
  udp.Recv();
  udp.GetRecv(state);
  std::cout << "leg height" << state.footRaiseHeight << std::endl;
  for (int i = 0; i < N_JOINTS; i++) { // first try with front legs
    qSignal_.push_back(state.motorState[i].q);
  }
}

bool HighLevel::UDPSend()
{
  udp.SetSend(cmd);
  udp.Send();
  return true;

}

bool HighLevel::RobotControl()
{
  motiontime += 1;

  cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0]  = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;

  for (int i = 0; i < int(SIGNAL_LENGTH / MOVEMENT_DURATION); i++) {
    if (motiontime >= i * MOVEMENT_DURATION + i * PAUSE_DURATION + 1 && motiontime < (i+1) * MOVEMENT_DURATION + i * PAUSE_DURATION + 1) {
      if (motiontime == i * MOVEMENT_DURATION + i * PAUSE_DURATION + 1) {
        // when in movement mode, update the scheme first
        updateMovementScheme(i);
        std::cout << "cmdVelAng:  " << cmdVelAng_ << " cmdVelMag: " << cmdVelMag_ << " footHeight: " << footHeight_ << " bodyHeight: " << bodyHeight_ << std::endl;

      }
      cmd.mode = 2;
      cmd.gaitType = 1;
      cmd.velocity[0] = cmdVelMag_ * cmdVelX_; // -1  ~ +1
      cmd.velocity[1] = cmdVelMag_ * cmdVelY_; // -1  ~ +1
      cmd.footRaiseHeight = footHeight_;
      cmd.bodyHeight = bodyHeight_;
    }
    if (motiontime >= (i+1) * MOVEMENT_DURATION + i * PAUSE_DURATION + 1 && motiontime < (i+1) * MOVEMENT_DURATION + (i+1) * PAUSE_DURATION + 1) {
      cmd.mode = 0;
      cmd.velocity[0] = 0;
    }
  }

  if (motiontime >= SIGNAL_LENGTH) {
    saveSignalAsFile();
    repeatCheck();
    exit(-1);
  }

  return true;
}

void HighLevel::updateMovementScheme(int i) {
  // TODO: add changing body height, leg height, velocity magnitude
  cmdVelAng_ = motionParamMat_(i, 0);
  cmdVelX_ = sin(cmdVelAng_);
  cmdVelY_ = -cos(cmdVelAng_);

  cmdVelMag_ = motionParamMat_(i, 1);

  footHeight_ = motionParamMat_(i, 2);

  bodyHeight_ = motionParamMat_(i, 3);
}

void HighLevel::initMotionScheme() {
  for (int l = 1; l < 2; l++) {
    bodyHeight_ = (l - 1) * 0.1; // -1, ..., 1

    for(int i = 4; i < 5; i++) {
      footHeight_ = (i - 2) * 0.1; // -2, -1, ..., 1, 2

      for(int j = 0; j < 1; j++) { // change this one-by-one
        velMag_ = (j + 1) * 0.2; // 1, 2, ...

        for(int k = 0; k < 8; k++) {
          if (k % 2 != 0) { // return trip
            velDir_ += M_PI;
          } else {
            velDir_ = k * M_PI / 8; // 0, 1, ...
          }

          motionParam_.push_back(velDir_);
          motionParam_.push_back(velMag_);
          motionParam_.push_back(footHeight_);
          motionParam_.push_back(bodyHeight_);

        }
      }
    }
  }

  // change list into eigen matrix
  motionParamMat_ = Eigen::Map<Eigen::Matrix<float, 8, 4, Eigen::RowMajor>>(motionParam_.data());
}

void HighLevel::saveSignalAsFile() {
  // change list into eigen matrix
  std::cout << "qSignal_ length" << qSignal_.size() << std::endl;
  qSignalMat_ = Eigen::Map<Eigen::Matrix<float, SIGNAL_LENGTH, N_JOINTS, Eigen::RowMajor>>(qSignal_.data());
  ofstream outFile;
  outFile.open("/home/zerenluo/unitree_ros_ws/src/Go1-QP-MPC-Controller/src/data_collection/data/qSineSignal.txt");
  outFile << qSignalMat_;
  outFile.close();
}

void HighLevel::repeatCheck() {
  std::vector<Eigen::VectorXf> vec;
  for (int i = 0; i < qSignalMat_.rows(); i++) {
    vec.push_back(qSignalMat_.row(i));
  }

  auto it = std::unique(vec.begin(), vec.end());
  vec.erase(it, vec.end());
//  Eigen::Matrix<float, vec.size(), N_JOINTS> qSignalMatClean;
  qSignalMat_.resize(vec.size(), N_JOINTS);
  for (int i = 0; i < vec.size(); i++) {
    qSignalMat_.row(i) = vec[i];
  }

  // save clean data
  ofstream outFile;
  outFile.open("/home/zerenluo/unitree_ros_ws/src/Go1-QP-MPC-Controller/src/data_collection/data/qSineSignalClean.txt");
  outFile << qSignalMat_;
  outFile.close();

}
