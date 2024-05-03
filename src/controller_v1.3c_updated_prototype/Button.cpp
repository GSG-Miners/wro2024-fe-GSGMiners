/**
 * @file Button.cpp
 * @brief Implementation of the Button class for debounced button interaction.
 */

#include "Button.h"

/**
 * @brief Constructs a Button object associated with a specific hardware pin.
 *
 * This constructor initializes a Button instance, setting up the pin which the button
 * is connected to. The pin is stored for use in other methods to read and manage the
 * button state.
 *
 * @param pin The microcontroller pin connected to the button.
 */
Button::Button(pin_size_t pin)
    : pin(pin) {}

/**
 * @brief Destructor for the Button class.
 *
 * Performs any necessary cleanup for the Button instance. This is particularly important
 * in environments where the Button class may allocate resources that need to be explicitly
 * released.
 */
Button::~Button() {}

/**
 * @brief Prepares the button for operation by setting initial conditions.
 *
 * Configures the button's hardware pin as an input with an internal pull-up resistor,
 * sets a default debounce delay to prevent false triggering from switch bounce, and
 * enables the button to start accepting input.
 */
void Button::begin()
{
  this->debounce_delay = DEBOUNCE_DELAY;
  pinMode(this->pin, INPUT_PULLUP);
  this->enabled = true;
}

/**
 * @brief Disables the button to stop processing its input.
 *
 * Marks the button as disabled within the software, which will prevent any further
 * input processing until the button is re-enabled.
 */
void Button::end()
{
  this->enabled = false;
}

/**
 * @brief Customizes the debounce delay to suit specific hardware or requirements.
 *
 * Allows the debounce delay to be set to a custom value, which can be adjusted based
 * on the characteristics of the physical button or the needs of the application.
 *
 * @param debounce_delay The desired debounce delay in milliseconds.
 */
void Button::setDebounceDelay(uint8_t debounce_delay)
{
  if (!this->enabled)
    return;

  this->debounce_delay = debounce_delay;
}

/**
 * @brief Reads and debounces the button state, updating internal counters.
 *
 * Reads the current state of the button, applies debouncing logic to ensure stability,
 * and updates internal counters based on the configured counting mode. This method is
 * typically called repeatedly within a loop to continuously update the button state.
 *
 * @return The debounced state of the button (HIGH or LOW).
 */
bool Button::readState()
{
  if (!this->enabled)
    return 0;

  if (this->updating_location != LOCATION_B && this->updating_location != LOCATION_C && this->updating_location != LOCATION_D)
  {
    this->updating_location = LOCATION_A;
  }

  static bool last_flickerable_state;
  static unsigned long last_debounce_time;

  this->current_state = digitalRead(this->pin);

  // Check if the button state has changed.
  if (this->current_state != last_flickerable_state)
  {
    last_debounce_time = millis();
    last_flickerable_state = this->current_state;
  }

  // Update the steady state after the debounce delay has passed.
  if (millis() - last_debounce_time > this->debounce_delay)
  {
    this->last_steady_state = this->current_state;
  }

  // Increment the press count based on the counting mode.
  switch (this->counting_mode)
  {
  case CHANGE: // Count every state change.
  {
    if (this->last_steady_state != this->current_state)
    {
      this->count++;
    }
  }
  break;
  case RISING: // Count only on rising edge (button press).
  {
    if (this->last_steady_state == LOW && this->current_state == HIGH)
    {
      this->count++;
    }
    break;
  }
  default: // Count only on falling edge (button release).
  {
    if (this->last_steady_state == HIGH && this->current_state == LOW)
    {
      this->count++;
    }
  }
  break;
  }

  return this->last_steady_state;
}

/**
 * @brief Determines if the button has been pressed.
 *
 * Evaluates the button's state transitions to identify a press action. A press is
 * recognized when the button state changes from HIGH to LOW, considering the debounce
 * logic to avoid false positives.
 *
 * @return True if the button has been pressed, false otherwise.
 */
bool Button::isPressed()
{
  if (!this->enabled)
    return 0;

  if (this->updating_location != LOCATION_A && this->updating_location != LOCATION_C && this->updating_location != LOCATION_D)
  {
    this->updating_location = LOCATION_B;
    this->readState();
  }

  if (this->last_steady_state == HIGH && this->current_state == LOW)
    return true;
  else
    return false;
}

/**
 * @brief Determines if the button has transitioned to a released state.
 *
 * This method checks the current and previous states of the button to ascertain if it has been
 * released. It is particularly useful in scenarios where the action is triggered upon release rather
 * than press. The method ensures that the button's state is up-to-date by invoking `readState()`
 * before making the determination.
 *
 * @return True if the button has been released since the last state check, false otherwise.
 */
bool Button::isReleased()
{
  if (!this->enabled)
    return 0;

  if (this->updating_location != LOCATION_A && this->updating_location != LOCATION_B && this->updating_location != LOCATION_D)
  {
    this->updating_location = LOCATION_C;
    this->readState();
  }
}

/**
 * @brief Retrieves the number of button presses according to a specified counting mode.
 *
 * This function tallies the number of button press events based on the counting mode provided.
 * It supports different modes such as counting all state changes, only on rising edges (button press),
 * or only on falling edges (button release). The count is maintained internally and is updated each
 * time this method is called.
 *
 * @param counting_mode Specifies the type of button press events to count: CHANGE, RISING, or FALLING.
 * @return The count of button presses as per the specified counting mode.
 */
uint8_t Button::readCount(uint8_t counting_mode)
{
  if (!this->enabled)
    return 0;

  this->counting_mode = counting_mode;

  if (this->updating_location != LOCATION_A && this->updating_location != LOCATION_B && this->updating_location != LOCATION_C)
  {
    this->updating_location = LOCATION_D;
    this->readState();
  }

  return this->count;
}

/**
 * @brief Obtains the count of button presses using the FALLING edge as the default counting mode.
 *
 * This convenience method defaults to counting the number of button releases. It leverages the
 * `readCount(uint8_t counting_mode)` method with FALLING mode to track the number of times the
 * button has been released, which is a common requirement in user interfaces.
 *
 * @return The total count of button release events.
 */
uint8_t Button::readCount()
{
  this->readCount(FALLING);
}