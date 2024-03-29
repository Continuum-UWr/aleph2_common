#include <libudev.h>

#include <cmath>
#include <cstring>
#include <cctype>

#include "SDL2/SDL.h"
#include "SDL2/SDL_events.h"
#include "SDL2/SDL_joystick.h"

#include "rclcpp/clock.hpp"

#include "joystick.hpp"
#include "rostalker.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#define DEADZONE 0.10f

JoystickManager::JoystickManager(
  const char * hostname,
  std::shared_ptr<JoystickHandler> handler,
  rclcpp::Logger logger)
: hostname_(hostname),
  handler_(handler),
  logger_(logger),
  clock_(RCL_SYSTEM_TIME),
  devices_(new DeviceMap())
{
  SDL_Init(SDL_INIT_JOYSTICK);
  handler_->setDeviceMap(devices_);

  std::string package_path = ament_index_cpp::get_package_share_directory("input_manager");
  std::string yaml_file = package_path + "/data/joystick_mapping.yaml";
  joystick_mapping_ = YAML::LoadFile(yaml_file);
}

inline float axisValue(float value)
{
  float axis_value = value / (value < 0 ? 32768.0f : 32767.0f);

  if (std::abs(axis_value) > DEADZONE) {
    axis_value -= (std::signbit(axis_value) ? -1.0f : 1.0f) * DEADZONE;
    axis_value /= (1.0f - DEADZONE);
  } else {
    axis_value = 0.0f;
  }
  return axis_value;
}

inline bool isJoystickEvent(const SDL_Event & event)
{
  return event.type >= SDL_JOYAXISMOTION && event.type < SDL_CONTROLLERAXISMOTION;
}

void JoystickManager::start()
{
  active_ = true;
  thread_ = new std::thread(&JoystickManager::threadLoop, this);
}

void JoystickManager::stop()
{
  active_ = false;
  thread_->join();
  delete thread_;
}

void JoystickManager::threadLoop()
{
  while (active_) {
    SDL_Event event;
    if (!SDL_WaitEventTimeout(&event, 100)) {
      continue;
    }

    // We are only interested in joystick events
    if (!isJoystickEvent(event)) {continue;}

    RCLCPP_DEBUG(logger_, "Received event type %d", event.type);

    if (event.type == SDL_JOYDEVICEADDED) {
      newDevice(event.jdevice.which);
      continue;
    }

    // The joystick instance id is at the same offset for all event types
    int joy_id = event.jdevice.which;

    if (devices_->find(joy_id) == devices_->end()) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000, "Received an event for a lost device");
      continue;
    }

    auto dev = (*devices_)[joy_id];
    std::lock_guard<std::mutex> dev_lock(dev->mutex);

    switch (event.type) {
      case SDL_JOYAXISMOTION:
        dev->axes[event.jaxis.axis] = axisValue(static_cast<float>(event.jaxis.value));
        break;

      case SDL_JOYBUTTONDOWN:
        dev->buttons[event.jbutton.button] = true;
        dev->presses.push_back(event.jbutton.button);
        break;

      case SDL_JOYBUTTONUP:
        dev->buttons[event.jbutton.button] = false;
        dev->releases.push_back(event.jbutton.button);
        break;

      case SDL_JOYHATMOTION: {
          int hat_position = dev->buttons_c + event.jhat.which * 4;

          for (int i = hat_position; i < hat_position + 4; i++) {
            dev->buttons[i] = false;
          }

          if (event.jhat.value & SDL_HAT_UP) {
            dev->buttons[hat_position + 0] = true;
            dev->presses.push_back(hat_position + 0);
          }

          if (event.jhat.value & SDL_HAT_DOWN) {
            dev->buttons[hat_position + 1] = true;
            dev->presses.push_back(hat_position + 1);
          }

          if (event.jhat.value & SDL_HAT_LEFT) {
            dev->buttons[hat_position + 2] = true;
            dev->presses.push_back(hat_position + 2);
          }

          if (event.jhat.value & SDL_HAT_RIGHT) {
            dev->buttons[hat_position + 3] = true;
            dev->presses.push_back(hat_position + 3);
          }

          break;
        }

      case SDL_JOYDEVICEREMOVED:
        deviceLost(joy_id);
        break;
    }
  }
}

std::optional<std::string> JoystickManager::getJoystickSerial(SDL_Joystick * joy)
{
  struct udev * udev;
  struct udev_enumerate * enumerate;
  struct udev_list_entry * u_devices, * dev_list_entry;
  struct udev_device * u_device;
  const char * serial_id = nullptr;

  std::string path = SDL_JoystickPath(joy);
  RCLCPP_DEBUG_STREAM(logger_, "Joystick path: " << path);
  /* Create the udev object */
  udev = udev_new();
  if (!udev) {
    RCLCPP_ERROR(logger_, "Couldn't create new udev context");
    exit(1);
  }

  /* Create a list of the devices in the 'input' subsystem. */
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "input");
  udev_enumerate_scan_devices(enumerate);
  u_devices = udev_enumerate_get_list_entry(enumerate);
  udev_list_entry_foreach(dev_list_entry, u_devices) {
    /* Get device path in the sys system */
    const char * sys_path = udev_list_entry_get_name(dev_list_entry);
    /* Create new udev device */
    u_device = udev_device_new_from_syspath(udev, sys_path);

    RCLCPP_DEBUG(logger_, "Device's syspath: %s", sys_path);

    if (!u_device) {
      RCLCPP_DEBUG(logger_, "Couldn't create new udev device: %s", sys_path);
      udev_device_unref(u_device);
      continue;
    }

    /* Get '/dev' device path */
    const char * dev_path = udev_device_get_devnode(u_device);
    if (!dev_path) {
      RCLCPP_DEBUG(logger_, "Couldn't get the device's devnode");
      udev_device_unref(u_device);
      continue;
    }

    RCLCPP_DEBUG(logger_, "Device's devnode path: %s", dev_path);

    std::string dev_path_str = std::string(dev_path);

    if (dev_path_str != path) {
      RCLCPP_DEBUG(logger_, "Devnode path does not match joystick path, skipping..");
      udev_device_unref(u_device);
      continue;
    }

    /* Get device's usb parent */
    u_device = udev_device_get_parent_with_subsystem_devtype(u_device, "usb", "usb_device");

    if (!u_device) {
      RCLCPP_DEBUG(logger_, "Couldn't get device's usb parent");
      udev_device_unref(u_device);
      continue;
    }

    serial_id = udev_device_get_sysattr_value(u_device, "serial");
    if (!serial_id) {
      RCLCPP_DEBUG(logger_, "Couldn't get device's 'serial' attribute");
      continue;
    }

    udev_device_unref(u_device);
    break;
  }
  /* Free the enumerator object */
  udev_enumerate_unref(enumerate);
  /* Free the udev context */
  udev_unref(udev);

  return serial_id ? std::optional<std::string>{serial_id} : std::nullopt;
}

std::string JoystickManager::getUniqueJoystickName(SDL_Joystick * joy)
{
  auto serial_id = getJoystickSerial(joy);
  auto joy_id = SDL_JoystickInstanceID(joy);
  auto joy_name = SDL_JoystickName(joy);
  auto joy_vendor = SDL_JoystickGetVendor(joy);
  auto joy_product = SDL_JoystickGetProduct(joy);

  RCLCPP_INFO(
    logger_, "Joystick name: '%s', vendor id: %04x, product id: %04x, serial id: '%s'",
    joy_name, joy_vendor, joy_product, serial_id.value_or("<UNKNOWN>").c_str());

  if (serial_id) {
    auto name = joystick_mapping_[serial_id.value()];
    if (name) {
      return name.as<std::string>();
    }
  }

  // No predefined unique name for this joystick
  // Create human-readable unique name from joystick name, instance id and the hostname
  std::string name = std::string(joy_name) + "_" + std::to_string(joy_id) + "_" + hostname_;

  std::replace_if(
    name.begin(), name.end(),
    [](char x) -> bool {
      return !(std::isalnum(x) || x == '_');
    }, '_');

  return name;
}

void JoystickManager::newDevice(int dev_id)
{
  auto joy = SDL_JoystickOpen(dev_id);
  auto dev = std::make_shared<Device>();
  auto joy_id = SDL_JoystickInstanceID(joy);

  RCLCPP_INFO(logger_, "New joystick connected, instance id: %d", joy_id);

  dev->axes.resize(SDL_JoystickNumAxes(joy));
  dev->buttons_c = SDL_JoystickNumButtons(joy);
  dev->buttons.resize(
    SDL_JoystickNumButtons(joy) +
    SDL_JoystickNumHats(joy) * 4);
  dev->name = getUniqueJoystickName(joy);

  for (int i = 0; i < dev->buttons_c; i++) {
    dev->buttons[i] = SDL_JoystickGetButton(joy, i);
  }
  for (unsigned int i = 0; i < dev->axes.size(); i++) {
    dev->axes[i] = axisValue(SDL_JoystickGetAxis(joy, i));
  }

  devices_->insert({joy_id, dev});
  handler_->registerDevice(joy_id);
}

void JoystickManager::deviceLost(int joy_id)
{
  handler_->unregisterDevice(joy_id);
  devices_->erase(joy_id);
}
