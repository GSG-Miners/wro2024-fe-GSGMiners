/**
 * @file button.h
 * @brief Button class for managing button state and count.
 * @date 23rd December 2023 - 3rd February 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <sys/_stdint.h>
#include <Arduino.h>

// Constants
const uint8_t DEFAULT_DEBOUNCE_DELAY = 50;

/**
 * @class Button
 * @brief Class to for managing button state and count.
 */
class Button
{
public:
  Button(uint8_t button_pin);
  ~Button();

  void update();
  void setDebounceDelay(uint8_t debounce_delay);
  bool isPressed();
  uint8_t getCount();

private:
  bool current_state_;       ///< The current state of the button (HIGH or LOW)
  bool last_unstable_state_; ///< The last state of the button that may change
  bool last_steady_state_;   ///< The last state of the button that was stable
  uint8_t button_pin_;       ///< The pin number of the button
  uint8_t debounce_delay_;   ///< The delay in milliseconds to ignore button flickering
  uint8_t count_;            ///< The number of times the button has been pressed
};

#endif // BUTTON_H