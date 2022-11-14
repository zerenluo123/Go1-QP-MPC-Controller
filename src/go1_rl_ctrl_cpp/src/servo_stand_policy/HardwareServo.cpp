//
// Created by zerenluo on 14.10.22.
//

#include "HardwareServo.hpp"

HardwareServo::HardwareServo():
safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007) {
  udp.InitCmdData(cmd);
  paramInit();
}

bool HardwareServo::UDPRecv()
{
  udp.Recv();
  udp.GetRecv(state);
  return true;
}

void HardwareServo::paramInit()
{
  for(int i=0; i<2; i++){
    cmd.motorCmd[i*3+0].mode = 0x0A;
    cmd.motorCmd[i*3+0].Kp = 10;
    cmd.motorCmd[i*3+0].dq = 0;
    cmd.motorCmd[i*3+0].Kd = 3;
    cmd.motorCmd[i*3+0].tau = 0;
    cmd.motorCmd[i*3+1].mode = 0x0A;
    cmd.motorCmd[i*3+1].Kp = 40;
    cmd.motorCmd[i*3+1].dq = 0;
    cmd.motorCmd[i*3+1].Kd = 4;
    cmd.motorCmd[i*3+1].tau = 0;
    cmd.motorCmd[i*3+2].mode = 0x0A;
    cmd.motorCmd[i*3+2].Kp = 20;
    cmd.motorCmd[i*3+2].dq = 0;
    cmd.motorCmd[i*3+2].Kd = 2;
    cmd.motorCmd[i*3+2].tau = 0;
  }

  for(int i=2; i<4; i++){
    cmd.motorCmd[i*3+0].mode = 0x0A;
    cmd.motorCmd[i*3+0].Kp = 10;
    cmd.motorCmd[i*3+0].dq = 0;
    cmd.motorCmd[i*3+0].Kd = 3;
    cmd.motorCmd[i*3+0].tau = 0;
    cmd.motorCmd[i*3+1].mode = 0x0A;
    cmd.motorCmd[i*3+1].Kp = 40;
    cmd.motorCmd[i*3+1].dq = 0;
    cmd.motorCmd[i*3+1].Kd = 4;
    cmd.motorCmd[i*3+1].tau = 0;
    cmd.motorCmd[i*3+2].mode = 0x0A;
    cmd.motorCmd[i*3+2].Kp = 30;
    cmd.motorCmd[i*3+2].dq = 0;
    cmd.motorCmd[i*3+2].Kd = 2;
    cmd.motorCmd[i*3+2].tau = 0;
  }

}

bool HardwareServo::send_cmd() {
  double pos[12] = {0.0, 0.6, -1.3, -0.0, 0.6, -1.3,
                    0.0, 0.6, -1.3, -0.0, 0.6, -1.3};

  // motor q command init
  for(int i=0; i<12; i++){
    cmd.motorCmd[i].q = state.motorState[i].q;
  }
  // change q command overtime
  moveAllPosition(pos, 2*30000);

  return true;
}

void HardwareServo::moveAllPosition(double* targetPos, double duration) {
  double pos[12] ,lastPos[12], percent;
  for(int j=0; j<12; j++) lastPos[j] = state.motorState[j].q;
  for(int i=1; i<=duration; i++){
    percent = (double)i/duration;
//    std::cout << "percent" <<  percent << std::endl;
    for(int j=0; j<12; j++){
      cmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent;
    }
  }

  safe.PositionLimit(cmd);
  safe.PowerProtect(cmd, state, 5);
//  safe.PositionProtect(cmd, state, -0.2);
  udp.SetSend(cmd);
  udp.Send();
}