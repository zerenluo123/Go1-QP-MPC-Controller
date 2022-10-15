//
// Created by zerenluo on 14.10.22.
//

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class HardwareServoSwitch
{
 public:
  HardwareServoSwitch(uint8_t level);
  bool UDPRecv();
  void RobotControl();

  void paramInit();
  bool send_cmd();
  void moveAllPosition(double* targetPos, double duration);

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};

  float dt = 0.002;     // 0.001~0.01
};