#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_events.h>

#include <list>
#include <cmath>
#include <functional>

#include <cctype>

#include "joystick.h"
#include "rostalker.h"

#define DEADZONE 0.10f

char hostname[HOST_NAME_MAX];

void JoystickManager::Init() {
    SDL_Init(SDL_INIT_JOYSTICK);
    gethostname(hostname, HOST_NAME_MAX);
}

void JoystickManager::Update() {
    SDL_Event event;

    while(SDL_PollEvent(&event)) {
        int dev_id = event.jdevice.which;
    
        switch(event.type) {
            case SDL_JOYAXISMOTION: {
                float axis_value =  (float)event.jaxis.value /
                (event.jaxis.value < 0 ? 32768.0f : 32767.0f);

                if(std::abs(axis_value) > DEADZONE) {
                    axis_value -= (std::signbit(axis_value) ? -1.0f : 1.0f) * DEADZONE;
                    axis_value /= (1.0f - DEADZONE);
                } else
                axis_value = 0.0f;
                
                devices[event.jdevice.which]->axes[event.jaxis.axis] = axis_value;
                break;
            }
                
            case SDL_JOYBUTTONDOWN:
                devices[event.jdevice.which]->buttons[event.jbutton.button] = true;
                devices[event.jdevice.which]->presses.push_back(event.jbutton.button);
                break;
            case SDL_JOYBUTTONUP:
                devices[event.jdevice.which]->buttons[event.jbutton.button] = false;
                devices[event.jdevice.which]->releases.push_back(event.jbutton.button);
                break;
                
            case SDL_JOYHATMOTION: {
                int hat_position = devices[dev_id]->buttons_c + event.jhat.which * 4;
                
                for(int i = hat_position; i < hat_position + 4; i++)
                    devices[dev_id]->buttons[i] = false;
                
                if ( event.jhat.value & SDL_HAT_UP ) {
                    devices[event.jdevice.which]->buttons[hat_position + 0] = true;
                    devices[event.jdevice.which]->presses.push_back(hat_position + 0);
                }

                if ( event.jhat.value & SDL_HAT_DOWN ) {
                    devices[event.jdevice.which]->buttons[hat_position + 1] = true;
                    devices[event.jdevice.which]->presses.push_back(hat_position + 1);
                }

                if ( event.jhat.value & SDL_HAT_LEFT ) {
                    devices[event.jdevice.which]->buttons[hat_position + 2] = true;
                    devices[event.jdevice.which]->presses.push_back(hat_position + 2);
                }

                if ( event.jhat.value & SDL_HAT_RIGHT ) {
                    devices[event.jdevice.which]->buttons[hat_position + 3] = true;
                    devices[event.jdevice.which]->presses.push_back(hat_position + 3);
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
    for(int i = 0; i < 16; i++)
        guid_short += guid.data[i] * i;

    state->axes.resize(SDL_JoystickNumAxes(dev));
    state->buttons_c = SDL_JoystickNumButtons(dev);
    state->buttons.resize(SDL_JoystickNumButtons(dev) + SDL_JoystickNumHats(dev) * 4);
    state->name = SDL_JoystickName(dev);
    state->name += std::string("_") + std::to_string(joy_id);
    //state->name += std::string("_") + std::to_string(guid_short);
    state->name += std::string("_") + std::string(hostname);

    std::replace_if(state->name.begin(), state->name.end(), 
        [](char x) -> bool {
            return !( isalnum(x) || x == '_' );
        }, '_');

    std::fill(state->axes.begin(), state->axes.end(), 0);
    std::fill(state->buttons.begin(), state->buttons.end(), 0);

    if(inst_id == devices.size())
        devices.resize(devices.size() + 1);

    devices[inst_id] = state;
    RosTalker::Instance().RegisterDevice(state);
}

void JoystickManager::DeviceLost(int id) {
    devices[id].reset();
}
