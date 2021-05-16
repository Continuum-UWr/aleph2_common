#include <list>
#include <sstream>

#include <input_manager/DeviceList.h>
#include <input_manager/Input.h>

#include "rostalker.h"

struct RosTalkerInternals {};

void RosTalker::Init(int argc, char **argv) {
  ros::init(argc, argv, "input_manager", ros::init_options::AnonymousName);

  n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pn = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
  pub = n->advertise<input_manager::DeviceList>("input/devices_list", 1, true);
  n_name = ros::this_node::getName();
  n_name = n_name.substr(1);
}

void RosTalker::RegisterDevice(std::weak_ptr<DeviceState> dev) {
  auto device_locked = dev.lock();

  ROS_INFO("Registered device: %s", device_locked->name.c_str());
  topics.push_back(std::make_tuple(
      pn->advertise<input_manager::Input>(device_locked->name, 10), dev));
  PublishDevices();
}

void RosTalker::Update() {
  ros::Rate loop_rate(30);
  bool send = false;
  topics.erase(
      std::remove_if(
          topics.begin(), topics.end(),
          [&send](std::tuple<ros::Publisher, std::weak_ptr<DeviceState>> entry)
              -> bool {
            if (std::get<1>(entry).expired()) {
              ROS_INFO("Device lost!");
              send = true;
              return true;
            }
            bool t = std::get<0>(entry).getNumSubscribers() > 0;
            auto device = std::get<1>(entry).lock();
            if (t != device->active) {
              device->active = t;
              send = true;
            }

            input_manager::Input msg;

            msg.axes = device->axes;
            msg.buttons = device->buttons;
            msg.buttons_pressed = std::move(device->presses);
            msg.buttons_released = std::move(device->releases);
            std::get<0>(entry).publish(msg);

            return false;
          }),
      topics.end());

  ros::spinOnce();
  if (send) {
    PublishDevices();
  }
  loop_rate.sleep();
}

void RosTalker::PublishDevices(bool shutdown) {
  input_manager::DeviceList msg;

  msg.node_name = n_name;
  if (!shutdown) {
    for (auto dev : topics) {
      auto device = std::get<1>(dev).lock();
      msg.devices_list.push_back(device->name);
      msg.active.push_back(device->active);
    }
  }

  pub.publish(msg);
}
