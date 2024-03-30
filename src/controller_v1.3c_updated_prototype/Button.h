/**
 * @file Button.h
 * @brief Header for Button class, which handles button debouncing and state reading.
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#include <stdint.h>
#ifndef BUTTON_H
#define BUTTON_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef EXTENDED_PIN_MODE
typedef uint32_t pin_size_t;
#else
typedef uint8_t pin_size_t;
#endif

#define LOCATION_A 1
#define LOCATION_B 2
#define DEBOUNCE_DELAY 50

class Button {
public:
  Button(pin_size_t pin);
  ~Button();

  void begin();
  void end();
  void setDebounceDelay(uint8_t debounce_delay);
  bool readState();
  uint8_t readCount();

private:
  bool enabled;
  pin_size_t pin;
  uint8_t updating_location;
  uint8_t debounce_delay;
  uint8_t count;
};

#endif