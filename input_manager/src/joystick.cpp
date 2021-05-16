#include <cmath>
#include <functional>
#include <list>

#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_joystick.h>

#include <ros/ros.h>
#include <cctype>

#include "joystick.h"
#include "rostalker.h"

#define DEADZONE 0.10f

char hostname[HOST_NAME_MAX];

void JoystickManager::Init() {
  SDL_Init(SDL_INIT_JOYSTICK);
  gethostname(hostname, HOST_NAME_MAX);
}

float AxisValue(float value) {
  float axis_value = value / (value < 0 ? 32768.0f : 32767.0f);

  if (std::abs(axis_value) > DEADZONE) {
    axis_value -= (std::signbit(axis_value) ? -1.0f : 1.0f) * DEADZONE;
    axis_value /= (1.0f - DEADZONE);
  } else
    axis_value = 0.0f;
  return axis_value;
}

void JoystickManager::Update() {
  SDL_Event event;

  while (SDL_PollEvent(&event)) {
    int dev_id = event.jdevice.which;
    if (devices.find(dev_id) == devices.end() &&
        event.type != SDL_JOYDEVICEADDED) {
      ROS_WARN_THROTTLE(1.0, "Received an event for lost device");
      continue;
    }

    switch (event.type) {
      case SDL_JOYAXISMOTION: {
        devices[dev_id]->axes[event.jaxis.axis] = AxisValue(event.jaxis.value);
        break;
      }

      case SDL_JOYBUTTONDOWN:
        devices[dev_id]->buttons[event.jbutton.button] = true;
        devices[dev_id]->presses.push_back(event.jbutton.button);
        break;
      case SDL_JOYBUTTONUP:
        devices[dev_id]->buttons[event.jbutton.button] = false;
        devices[dev_id]->releases.push_back(event.jbutton.button);
        break;

      case SDL_JOYHATMOTION: {
        int hat_position = devices[dev_id]->buttons_c + event.jhat.which * 4;

        for (int i = hat_position; i < hat_position + 4; i++)
          devices[dev_id]->buttons[i] = false;

        if (event.jhat.value & SDL_HAT_UP) {
          devices[dev_id]->buttons[hat_position + 0] = true;
          devices[dev_id]->presses.push_back(hat_position + 0);
        }

        if (event.jhat.value & SDL_HAT_DOWN) {
          devices[dev_id]->buttons[hat_position + 1] = true;
          devices[dev_id]->presses.push_back(hat_position + 1);
        }

        if (event.jhat.value & SDL_HAT_LEFT) {
          devices[dev_id]->buttons[hat_position + 2] = true;
          devices[dev_id]->presses.push_back(hat_position + 2);
        }

        if (event.jhat.value & SDL_HAT_RIGHT) {
          devices[dev_id]->buttons[hat_position + 3] = true;
          devices[dev_id]->presses.push_back(hat_position + 3);
        }

        break;
      }

      case SDL_JOYDEVICEADDED:
        NewDevice(event.jdevice.which);
        break;
      case SDL_JOYDEVICEREMOVED:
        DeviceLost(event.jdevice.which);
        break;
    }
  }
}

void JoystickManager::NewDevice(int joy_id) {
  auto dev = SDL_JoystickOpen(joy_id);
  auto state = std::make_shared<DeviceState>();
  auto inst_id = SDL_JoystickInstanceID(dev);
  auto guid = SDL_JoystickGetDeviceGUID(joy_id);
  uint16_t guid_short = 0;
  for (int i = 0; i < 16; i++) guid_short += guid.data[i] * i;

  state->axes.resize(SDL_JoystickNumAxes(dev));
  state->buttons_c = SDL_JoystickNumButtons(dev);
  state->buttons.resize(SDL_JoystickNumButtons(dev) +
                        SDL_JoystickNumHats(dev) * 4);
  state->name = SDL_JoystickName(dev);
  state->name += std::string("_") + std::to_string(joy_id);
  // state->name += std::string("_") + std::to_string(guid_short);
  state->name += std::string("_") + std::string(hostname);

  std::replace_if(
      state->name.begin(), state->name.end(),
      [](char x) -> bool { return !(isalnum(x) || x == '_'); }, '_');

  for (int i = 0; i < state->buttons_c; i++) {
    state->buttons[i] = SDL_JoystickGetButton(dev, i);
  }
  for (unsigned int i = 0; i < state->axes.size(); i++) {
    state->axes[i] = AxisValue(SDL_JoystickGetAxis(dev, i));
  }

  devices.insert({inst_id, state});
  RosTalker::Instance().RegisterDevice(state);
}

void JoystickManager::DeviceLost(int id) {
  ROS_INFO("Removed device: %s", devices[id]->name.c_str());
  devices[id].reset();
  devices.erase(id);
}
