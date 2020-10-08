#include <iostream>
#include <signal.h>

#include "joystick.h"

int main(int argc, char**argv) {
    RosTalker::Instance().Init(argc, argv);
    JoystickManager::Instance().Init();

    while(ros::ok()){
        JoystickManager::Instance().Update();
        RosTalker::Instance().Update();
    }

    return 0;
}
