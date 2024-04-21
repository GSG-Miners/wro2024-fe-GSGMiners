/**
 * @file Controlling.cpp
 * @brief Implementation of the DoubleSetpointController class.
 */

#include "Controlling.h"

/**
 * @brief Constructor that initializes the controller with a specific race.direction.
 *
 * Creates a new instance of the DoubleSetpointController class, setting the initial control
 * race.direction and preparing the controller for use.
 *
 * @param race.direction Initial race.direction of control.
 */
DoubleSetpointController::DoubleSetpointController(ControllerDirection direction)
    : direction(direction) {}

/**
 * @brief Destructs the DoubleSetpointController object.
 */
DoubleSetpointController::~DoubleSetpointController() {}

/**
 * @brief Initializes the controller for operation.
 *
 * Sets the internal flag to enable the controller, allowing it to start processing inputs.
 */
void DoubleSetpointController::begin()
{
  this->enabled = true;
}

/**
 * @brief Shuts down the controller.
 *
 * Clears the internal flag to disable the controller, preventing it from processing inputs.
 */
void DoubleSetpointController::end()
{
  this->enabled = false;
}

/**
 * @brief Configures the race.direction of control.
 *
 * Sets the control race.direction to either DIRECT or REVERSE, affecting how the controller
 * responds to input values relative to the setpoint.
 *
 * @param race.direction The desired race.direction of control.
 */
void DoubleSetpointController::setDirection(ControllerDirection direction)
{
  if (!this->enabled)
    return;

  this->direction = direction;
}

/**
 * @brief Assigns a new setpoint for the controller.
 *
 * Updates the setpoint against which input values are compared to determine the control output.
 *
 * @param setpoint The target setpoint value.
 */
void DoubleSetpointController::setSetpoint(int16_t setpoint)
{
  if (!this->enabled)
    return;

  this->setpoint = setpoint;
}

/**
 * @brief Sets the output states for various input ranges.
 *
 * Configures the output states that the controller will use when the input is below, within,
 * or above the setpoint range, considering the hysteresis.
 *
 * @param low_state Output state for input values below the setpoint range.
 * @param medium_state Output state for input values within the setpoint range.
 * @param high_state Output state for input values above the setpoint range.
 */
void DoubleSetpointController::setStates(uint16_t low_state, uint16_t medium_state, uint16_t high_state)
{
  if (!this->enabled)
    return;

  this->low_state = low_state;
  this->medium_state = medium_state;
  this->high_state = high_state;
}

/**
 * @brief Defines the hysteresis width around the setpoint.
 *
 * Sets the hysteresis value which creates a range around the setpoint where the output
 * will be set to the medium state, preventing rapid toggling near the setpoint.
 *
 * @param hysteresis The width of the hysteresis range.
 */
void DoubleSetpointController::setHysteresis(uint8_t hysteresis)
{
  if (!this->enabled)
    return;

  this->hysteresis = hysteresis;
}

/**
 * @brief Calculates the output based on the current input and setpoint.
 *
 * Processes the input value and determines the appropriate output state based on the
 * setpoint, hysteresis, and control race.direction.
 *
 * @param input The current input value to the controller.
 * @return The calculated output state.
 */
int16_t DoubleSetpointController::getOutput(uint16_t input)
{
  if (!this->enabled)
    return 0;

  int16_t output;

  if (input >= this->setpoint - this->hysteresis && input <= this->setpoint + this->hysteresis)
  {
    output = this->medium_state;
  }
  else if (input < this->setpoint - this->hysteresis)
  {
    if (this->direction == ControllerDirection::DIRECT)
    {
      output = this->high_state;
    }
    else if (this->direction == ControllerDirection::REVERSE)
    {
      output = this->low_state;
    }
  }
  else if (input > this->setpoint + this->hysteresis)
  {
    if (this->direction == ControllerDirection::DIRECT)
    {
      output = this->low_state;
    }
    else if (this->direction == ControllerDirection::REVERSE)
    {
      output = this->high_state;
    }
  }

  return output;
}