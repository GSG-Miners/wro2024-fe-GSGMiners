/**
 * @file Button.h
 * @brief Header file for the Button class, which handles button debouncing and state reading.
 *
 * The Button class provides methods for managing button input, including debouncing to
 * prevent false readings due to mechanical noise and counting button presses based on
 * different modes (CHANGE, RISING, or FALLING). It abstracts the low-level details of
 * button handling, making it easier to work with buttons in Arduino projects.
 *
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under the MIT License.
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <sys/_stdint.h>
#include <api/Common.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LOCATION_A 1
#define LOCATION_B 2
#define LOCATION_C 3
#define LOCATION_D 4

#define DEBOUNCE_DELAY 50

class Button {
public:
  Button(pin_size_t pin);
  ~Button();

  void begin();
  void end();
  void setDebounceDelay(uint8_t debounce_delay);
  bool readState();
  bool isPressed();
  bool isReleased();
  uint8_t readCount(uint8_t counting_mode);
  uint8_t readCount();

private:
  bool enabled;
  bool last_steady_state;
  bool current_state;
  pin_size_t pin;
  uint8_t updating_location;
  uint8_t debounce_delay;
  uint8_t count;
  uint8_t counting_mode;
};

#endif  // BUTTON_H