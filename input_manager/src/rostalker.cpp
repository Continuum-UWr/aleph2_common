#include <chrono>

#include "input_manager/msg/device_info.hpp"
#include "input_manager/msg/device_list.hpp"
#include "input_manager/msg/input.hpp"

#include "rostalker.hpp"

using namespace std::literals::chrono_literals;

RosTalker::RosTalker(const std::string & node_name)
: Node(node_name)
{
  this->declare_parameter("update_period_ms", 20);

  auto qos = rclcpp::QoS(1)
    .reliable()
    .transient_local();

  device_list_pub_ = this->create_publisher<input_manager::msg::DeviceList>("input/devices", qos);

  int update_period_ms;
  this->get_parameter("update_period_ms", update_period_ms);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms),
    std::bind(&RosTalker::update, this)
  );
}

void RosTalker::registerDevice(int joy_id)
{
  std::lock_guard<std::mutex> publishers_lock(publishers_mutex_);

  auto dev = (*devices_)[joy_id];

  publishers_.insert(
    {joy_id,
      this->create_publisher<input_manager::msg::Input>("input/data/" + dev->name, 10)});

  RCLCPP_INFO(get_logger(), "Registered device %d: %s", joy_id, dev->name.c_str());

  publishDevices();
}

void RosTalker::unregisterDevice(int joy_id)
{
  std::lock_guard<std::mutex> publishers_lock(publishers_mutex_);

  auto dev = (*devices_)[joy_id];

  publishers_.erase(joy_id);

  RCLCPP_INFO(get_logger(), "Unregistered device %d: %s", joy_id, dev->name.c_str());

  publishDevices();
}

void RosTalker::publishDevices()
{
  input_manager::msg::DeviceList msg;

  msg.node_name = get_name();
  for (auto [joy_id, _] : publishers_) {
    auto dev = (*devices_)[joy_id];
    input_manager::msg::DeviceInfo info;
    info.name = dev->name;
    info.num_buttons = dev->buttons.size();
    info.num_axes = dev->axes.size();
    msg.devices.push_back(info);
    msg.active.push_back(dev->active);
  }

  device_list_pub_->publish(msg);
}

void RosTalker::shutdown()
{
  std::lock_guard<std::mutex> publishers_lock(publishers_mutex_);

  publishers_.clear();
  publishDevices();
}

void RosTalker::update()
{
  std::lock_guard<std::mutex> publishers_lock(publishers_mutex_);

  bool devices_changed = false;

  for (auto [joy_id, publisher] : publishers_) {
    auto dev = (*devices_)[joy_id];
    std::lock_guard<std::mutex> dev_lock(dev->mutex);

    input_manager::msg::Input msg;

    msg.axes = dev->axes;
    msg.buttons = dev->buttons;
    msg.buttons_pressed = std::move(dev->presses);
    msg.buttons_released = std::move(dev->releases);

    publisher->publish(msg);

    const bool is_active = publisher->get_subscription_count() > 0;
    if (is_active != dev->active) {
      dev->active = is_active;
      devices_changed = true;
    }
  }

  if (devices_changed) {
    publishDevices();
  }
}
