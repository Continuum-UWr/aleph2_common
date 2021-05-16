
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "rostalker.h"

class JoystickManager {
 public:
  static JoystickManager& Instance() {
    static JoystickManager inst;
    return inst;
  }

  void Init();
  void Update();

  JoystickManager(JoystickManager const&) = delete;
  void operator=(JoystickManager const&) = delete;

 private:
  JoystickManager(){};

  void NewDevice(int id);
  void DeviceLost(int id);
  std::unordered_map<int, std::shared_ptr<DeviceState>> devices;
};

#endif /* JOYSTICK_H */
