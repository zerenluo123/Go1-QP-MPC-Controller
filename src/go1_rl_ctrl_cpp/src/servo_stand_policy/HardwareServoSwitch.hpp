//
// Created by zerenluo on 14.10.22.
//

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

class HardwareServoSwitch
{
 public:
  HardwareServoSwitch();
  bool UDPRecv();
  void RobotControl();

  void paramInit();
  bool send_cmd();
  void moveAllPosition(double* targetPos, double duration);

  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::UDP udp;
  UNITREE_LEGGED_SDK::LowCmd cmd = {0};
  UNITREE_LEGGED_SDK::LowState state = {0};

  float dt = 0.002;     // 0.001~0.01
};