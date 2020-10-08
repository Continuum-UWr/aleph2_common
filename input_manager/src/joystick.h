
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "rostalker.h"

#include <memory>
#include <vector>

class JoystickManager
{
    public:
        static JoystickManager& Instance()
        {
            static JoystickManager inst;
            return inst;
        }

        void Init();
        void Update();

        JoystickManager(JoystickManager const&) = delete;
        void operator=(JoystickManager const&)   = delete;

    private:
        JoystickManager() {};

        void NewDevice(int id);
        void DeviceLost(int id);
        std::vector<std::shared_ptr<DeviceState>> devices;

};

#endif /* JOYSTICK_H */

