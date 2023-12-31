/**
 * @file button.cpp
 * @brief Implementation of the Button class.
 */

#include <api/Common.h>
#include <sys/_stdint.h>
#include "button.h"

/**
 * @brief Constructor that initializes the button pin.
 * @param button_pin The pin number of the button.
 */
Button::Button(uint8_t button_pin) : button_pin_(button_pin)
{
  pinMode(button_pin_, INPUT_PULLUP);
  debounce_delay_ = DEFAULT_DEBOUNCE_DELAY;
}

/**
 * @brief Destructor that sets the button pin to OUTPUT.
 */
Button::~Button()
{
  pinMode(button_pin_, OUTPUT);
}

/**
 * @brief Updates the button state and count.
 */
void Button::update()
{
  unsigned long last_debounce_time;

  current_state_ = digitalRead(button_pin_);
  if (current_state_ != last_unstable_state_)
  {
    last_debounce_time = millis();
    last_unstable_state_ = current_state_;
  }
  if (millis() - last_debounce_time > debounce_delay_)
  {
    if (last_steady_state_ == HIGH && current_state_ == LOW)
    {
      count_++;
    }
    last_steady_state_ = current_state_;
  }
}

/**
 * @brief Sets the debounce delay for the button.
 * @param debounce_delay The delay in milliseconds to ignore button changes.
 */
void Button::setDebounceDelay(uint8_t debounce_delay)
{
  debounce_delay_ = debounce_delay;
}

/**
 * @brief Checks if the button is pressed.
 * @return True if the button is pressed, false otherwise.
 */
bool Button::isPressed()
{
  return !current_state_;
}

/**
 * @brief Returns the button press count.
 * @return The number of times the button has been pressed.
 */
uint8_t Button::getCount()
{
  return count_;
}