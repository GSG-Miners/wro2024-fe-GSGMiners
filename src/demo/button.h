/**
 * @file button.h
 * @brief Button class for managing button state and count.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 25th December 2023 by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
*/

#ifndef BUTTON_H
#define BUTTON_H

#include <sys/_stdint.h>
#include <Arduino.h>

// Constants
const uint8_t kDefaultDebounceDelay = 50;

/**
 * @class Button
 * @brief Class to for managing button state and count.
*/
class Button {
public:
  Button(uint8_t button_pin);
  ~Button();

  void update();
  void setDebounceDelay(uint8_t debounce_delay);
  bool isPressed();
  uint8_t getCount();

private:
  bool current_state_;
  bool last_flickerable_state_;
  bool last_steady_state_;
  uint8_t button_pin_;
  uint8_t debounce_delay_;
  uint8_t count_;
};

#endif