#pragma once

#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "input_manager/msg/device_list.hpp"
#include "input_manager/msg/input.hpp"

#include "joystick.hpp"

typedef std::unordered_map<int,
    rclcpp::Publisher<input_manager::msg::Input>::SharedPtr> PublisherMap;

class RosTalker : public rclcpp::Node, public JoystickHandler
{
public:
  RosTalker(const std::string & node_name);

  void setDeviceMap(std::shared_ptr<DeviceMap> devices) {devices_ = devices;}
  void registerDevice(int joy_id);
  void unregisterDevice(int joy_id);

  void publishDevices();
  void shutdown();

  RosTalker(RosTalker const &) = delete;
  void operator=(RosTalker const &) = delete;

private:
  void update();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<input_manager::msg::DeviceList>::SharedPtr device_list_pub_;
  std::shared_ptr<DeviceMap> devices_;
  PublisherMap publishers_;
  std::mutex publishers_mutex_;
};
