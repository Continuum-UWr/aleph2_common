#include <cmath>
#include <climits>

#include <unistd.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_joystick.h>

#include <cctype>

#include "rclcpp/clock.hpp"

#include "joystick.hpp"
#include "rostalker.hpp"

#define DEADZONE 0.10f

char hostname[HOST_NAME_MAX];

JoystickManager::JoystickManager(std::shared_ptr<JoystickHandler> handler, rclcpp::Logger logger)
: handler_(handler), logger_(logger), clock_(RCL_SYSTEM_TIME), devices_(new DeviceMap())
{
  SDL_Init(SDL_INIT_JOYSTICK);
  gethostname(hostname, HOST_NAME_MAX);

  handler_->setDeviceMap(devices_);
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

inline bool isJoystickEvent(const SDL_Event& event){
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
    if (!isJoystickEvent(event)) continue;

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

void JoystickManager::newDevice(int dev_id)
{
  auto joy = SDL_JoystickOpen(dev_id);
  auto dev = std::make_shared<Device>();
  auto joy_id = SDL_JoystickGetDeviceInstanceID(dev_id);

  dev->axes.resize(SDL_JoystickNumAxes(joy));
  dev->buttons_c = SDL_JoystickNumButtons(joy);
  dev->buttons.resize(
    SDL_JoystickNumButtons(joy) +
    SDL_JoystickNumHats(joy) * 4);
  dev->name = SDL_JoystickName(joy);
  dev->name += std::string("_") + std::to_string(joy_id);
  dev->name += std::string("_") + std::string(hostname);

  std::replace_if(
    dev->name.begin(), dev->name.end(),
    [](char x) -> bool {return !(std::isalnum(x) || x == '_');}, '_');

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
