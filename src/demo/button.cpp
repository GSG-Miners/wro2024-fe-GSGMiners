/**
 * @file button.cpp
 * @brief Implementation of the Button class.
*/

#include <api/Common.h>
#include <sys/_stdint.h>
#include "button.h"

/**
 * @brief Button constructor.
 * @param button_pin The pin connected to the button, where the current 
 * state is being read.
*/
Button::Button(uint8_t button_pin) : button_pin_(button_pin) {
  pinMode(button_pin_, INPUT_PULLUP);
  debounce_delay_ = 50;
}

/**
 * @brief Button destructor.
*/
Button::~Button() {
  pinMode(button_pin_, OUTPUT);
}

/**
 * @brief Updates the button state and count.
*/
void Button::update() {
  unsigned long last_debounce_time;

  current_state_ = digitalRead(button_pin_);
  if (current_state_ != last_flickerable_state_) {
    last_debounce_time = millis();
    last_flickerable_state_ = current_state_;
  }
  if (millis() - last_debounce_time > debounce_delay_) {
    if (last_steady_state_ == HIGH && current_state_ == LOW) {
      count_++;
    }
    last_steady_state_ = current_state_;
  }
}

/**
 * @brief Sets the debounce delay.
 * @param debounce_delay The desired debounce delay in milliseconds.
*/
void Button::setDebounceDelay(uint8_t debounce_delay) {
  debounce_delay_ = debounce_delay;
}

/**
 * @brief Checks if the button is pressed.
 * @return True if the button is pressed.
*/
bool Button::isPressed() {
  return !current_state_;
}

/**
 * @brief Returns the button press count.
 * @return The number of button presses.
*/
uint8_t Button::getCount() {
  return count_;
}