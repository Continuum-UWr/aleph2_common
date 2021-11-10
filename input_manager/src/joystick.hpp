#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <thread>
#include <mutex>

#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"

struct Device
{
  std::mutex mutex;
  std::string name;
  std::vector<float> axes;
  std::vector<bool> buttons;
  std::vector<int> presses;
  std::vector<int> releases;
  bool active;
  int buttons_c;
};

typedef std::unordered_map<int, std::shared_ptr<Device>> DeviceMap;

class JoystickHandler
{
public:
  virtual void setDeviceMap(std::shared_ptr<DeviceMap>) = 0;
  virtual void registerDevice(int joy_id) = 0;
  virtual void unregisterDevice(int joy_id) = 0;
};

class JoystickManager
{
public:
  JoystickManager() = delete;
  JoystickManager(JoystickManager const &) = delete;
  void operator=(JoystickManager const &) = delete;

  JoystickManager(
    const char * hostname,
    std::shared_ptr<JoystickHandler> handler,
    rclcpp::Logger logger);

  void start();
  void stop();

private:
  void threadLoop();
  void newDevice(int id);
  void deviceLost(int id);

  const std::string hostname_;
  std::shared_ptr<JoystickHandler> handler_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  std::shared_ptr<DeviceMap> devices_;
  std::thread * thread_;
  bool active_;
};
