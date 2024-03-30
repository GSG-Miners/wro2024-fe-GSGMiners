/**
 * @file Button.cpp
 * @brief Implementation of the Button class.
 */

#include "Button.h"

/**
 * @brief Constructor for Button class.
 * @param pin Pin connected to the button.
 */
Button::Button(pin_size_t pin)
  : pin(pin) {}

/**
 * @brief Destructor for Button class.
 */
Button::~Button() {}

/**
 * @brief Initializes the button.
 *
 * Sets the debounce delay and pin mode for the button.
 */
void Button::begin() {
  this->debounce_delay = DEBOUNCE_DELAY;
  pinMode(this->pin, INPUT_PULLUP);
  this->enabled = true;
}

/**
 * @brief Disables the button.
 */
void Button::end() {
  this->enabled = false;
}

/**
 * @brief Sets the debounce delay for the button.
 * @param debounce_delay The debounce delay in milliseconds.
 */
void Button::setDebounceDelay(uint8_t debounce_delay) {
  if (!this->enabled)
    return;

  this->debounce_delay = debounce_delay;
}

/**
 * @brief Reads the current state of the button.
 * @return True if the button is pressed, false otherwise.
 *
 * Implements debouncing logic to ensure stable button state reading.
 */
bool Button::readState() {
  if (!this->enabled)
    return 0;

  if (this->updating_location != LOCATION_B) {
    this->updating_location = LOCATION_A;
  }

  bool current_state;
  static bool last_flickerable_state;
  static bool last_steady_state;
  static unsigned long last_debounce_time;

  current_state = digitalRead(this->pin);

  // State changed due to noise or pressing
  if (current_state != last_flickerable_state) {
    last_debounce_time = millis();
    last_flickerable_state = current_state;
  }

  // Steady state is updated, depending on the debounce delay
  if (millis() - last_debounce_time > this->debounce_delay) {
    last_steady_state = current_state;
  }

  // Increment the count, if only FALLING is counted
  if (last_steady_state == HIGH && current_state == LOW) {
    this->count++;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Reads the count of button presses.
 * @return The count of button presses.
 *
 * This function also updates the button state if necessary.
 */
uint8_t Button::readCount() {
  if (!this->enabled)
    return 0;

  if (this->updating_location != LOCATION_A) {
    this->updating_location = LOCATION_B;
    this->readState();
  }

  return this->count;
}