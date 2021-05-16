#include <signal.h>
#include <iostream>

#include "joystick.h"

void MyShutDown(int sig) {
  RosTalker::Instance().PublishDevices(true);
  ros::shutdown();
}

int main(int argc, char** argv) {
  RosTalker::Instance().Init(argc, argv);
  JoystickManager::Instance().Init();

  signal(SIGINT, MyShutDown);

  while (ros::ok()) {
    JoystickManager::Instance().Update();
    RosTalker::Instance().Update();
  }

  return 0;
}
