/**
 * @file Controlling.cpp
 * @brief Implementation of the predefined control classes for managing various types of control
 * systems.
 */

#include <sys/_stdint.h>
#include "Controlling.h"

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @class  Controller
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Initializes the controller with a specified direction.
 *
 * Constructs a Controller object and sets the initial direction of control, either
 * DIRECT or REVERSE. It also initializes the update delay to a default value, preparing
 * the controller for operation.
 *
 * @param direction The initial direction of control for the controller.
 */
Controller::Controller(ControllerDirection direction)
    : controller_direction(direction), update_delay(10)
{
}

/**
 * @brief Destructor for the Controller class.
 *
 * Cleans up the Controller object when it is no longer needed, ensuring that any
 * allocated resources are properly released.
 */
Controller::~Controller()
{
}

/**
 * @brief Sets the target setpoint for the controller.
 *
 * Configures the controller with a desired setpoint value that it will attempt to
 * maintain or reach during operation.
 *
 * @param setpoint The target setpoint value for the controller.
 */
void Controller::setSetpoint(int16_t setpoint)
{
  if (!this->enabled)
    return;

  this->setpoint = setpoint;
}

/**
 * @brief Activates the controller.
 *
 * Prepares the controller for operation by setting its enabled status to true.
 */
void Controller::begin()
{
  this->enabled = true;
}

/**
 * @brief Deactivates the controller.
 *
 * Stops the controller from operating by setting its enabled status to false.
 */
void Controller::end()
{
  this->enabled = false;
}

/**
 * @brief Configures the operational direction of the controller.
 *
 * Adjusts the controller's direction of operation to either DIRECT or REVERSE,
 * affecting how it responds to deviations from the setpoint.
 *
 * @param direction The desired operational direction for the controller.
 */
void Controller::setDirection(ControllerDirection direction)
{
  if (!this->enabled)
    return;

  this->controller_direction = direction;
}

/**
 * @brief Sets the interval at which the controller updates.
 *
 * Defines the frequency, in milliseconds, at which the controller samples inputs
 * and adjusts outputs, allowing for fine-tuning of the control loop timing.
 *
 * @param sample_time The desired update interval for the controller.
 */
void Controller::setSampleTime(uint8_t sample_time)
{
  if (!this->enabled)
    return;

  this->update_delay = sample_time;
}

/**
 * @brief Visualizes the input, setpoint, and output values via the serial port.
 *
 * Sends the current input, setpoint, and output values of the controller to the
 * serial port for plotting or monitoring, aiding in tuning and diagnostics.
 */
void Controller::plotData()
{
  if (!this->enabled)
    return;

  if (Serial)
  {
    Serial.print(this->input);
    Serial.print(",");
    Serial.print(this->setpoint);
    Serial.print(",");
    Serial.println(this->output);
  }
}

/**
 * @brief Retrieves the current setpoint of the controller.
 *
 * Returns the value that the controller is currently trying to maintain or reach,
 * providing insight into the target state of the control system.
 *
 * @return The current setpoint value of the controller.
 */
int16_t Controller::getSetpoint()
{
  if (!this->enabled)
    return 0;

  return this->setpoint;
}

/**
 * @brief Computes and returns the output value based on the current input.
 *
 * Processes the given input according to the controller's logic and returns the
 * corresponding output value that should be applied to the system being controlled.
 *
 * @param input The current input value to the controller.
 * @return The computed output value from the controller.
 */
int16_t Controller::getOutput(int16_t input)
{
  if (!this->enabled)
    return 0;

  return this->output;
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @class  PIDController
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Initializes the PID controller with a specified direction.
 *
 * Constructs a PIDController object, inheriting from the Controller class, and sets
 * the initial direction of control. It also initializes the PID gains and output limits
 * to default values.
 *
 * @param direction The initial direction of control for the PID controller.
 */
PIDController::PIDController(ControllerDirection direction)
    : Controller(direction)
{
  this->p_gain = 1.00;
  this->i_gain = 1.00;
  this->d_gain = 1.00;
  this->min_output = 0;
  this->max_output = 180;
}

/**
 * @brief Destructor for the PIDController class.
 *
 * Ensures that any resources allocated to the PID controller are properly released.
 */
PIDController::~PIDController()
{
}

/**
 * @brief Sets the PID gains for the controller.
 *
 * Configures the proportional, integral, and derivative gains of the PID controller,
 * affecting its responsiveness and stability. Negative gains are not allowed and will
 * result in no change to the controller's configuration.
 *
 * @param proportional_gain The gain for the proportional term of the PID controller.
 * @param integral_gain The gain for the integral term of the PID controller.
 * @param derivative_gain The gain for the derivative term of the PID controller.
 */
void PIDController::tune(float proportional_gain, float integral_gain, float derivative_gain)
{
  if (proportional_gain < 0 || integral_gain < 0 || derivative_gain < 0 || !this->enabled)
  {
    return;
  }
  this->p_gain = proportional_gain;
  this->i_gain = integral_gain;
  this->d_gain = derivative_gain;

  if (this->controller_direction == ControllerDirection::REVERSE)
  {
    this->p_gain = -this->p_gain;
    this->i_gain = -this->i_gain;
    this->d_gain = -this->d_gain;
  }
}

/**
 * @brief Establishes the output range that the PID controller can produce.
 *
 * Sets the minimum and maximum output values that the PID controller is allowed to
 * generate, providing a mechanism to prevent excessive or insufficient control actions.
 *
 * @param min_output The lower bound of the controller's output range.
 * @param max_output The upper bound of the controller's output range.
 */
void PIDController::setLimits(int16_t min_output, int16_t max_output)
{
  if (!this->enabled)
    return;

  if (min_output < max_output)
  {
    this->min_output = min_output;
    this->max_output = max_output;
    this->limits_configured = true;
  }
  else
  {
    return;
  }
}

/**
 * @brief Calculates and returns the output value based on the provided input.
 *
 * This function computes the control output of a PID controller by taking the current
 * system input and applying the PID algorithm using the setpoint, gains, and previously
 * calculated error values. It ensures the output is within configured limits and updates
 * internal states for subsequent calls.
 *
 * @param input The current input value from the system being controlled.
 * @return The calculated control output to be applied to the system.
 */
int16_t PIDController::getOutput(int16_t input)
{
  if (!this->enabled)
    return 0;

  static unsigned long last_millis;
  static int16_t last_error_value, error_sum;
  int16_t error_value, error_differential;
  this->input = input;

  // Update the values, depending on the updating interval.
  if (millis() - last_millis > this->update_delay)
  {
    last_millis = millis();

    // Calculate the error, error-sum, and error-differential.
    error_value = this->setpoint - this->input;
    error_sum += error_value * max(1UL, (millis() - last_millis));
    if (this->limits_configured)
    {
      error_sum = constrain(error_sum, this->min_output, this->max_output);
    }
    error_differential = (error_value - last_error_value) / max(1UL, (millis() - last_millis));

    // Calculate the output. Constrain, if necessary.
    this->output = this->p_gain * error_value + this->i_gain * error_sum + this->d_gain * error_differential;
    if (this->limits_configured)
    {
      this->constrained_output = constrain(this->output, this->min_output, this->max_output);
    }

    last_error_value = error_value;
  }

  // Return the output.
  if (this->limits_configured)
  {
    return this->constrained_output;
  }
  else
  {
    return this->output;
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @class  DoubleSetpointController
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Initializes the double setpoint controller with a specified direction.
 *
 * Constructs a DoubleSetpointController object, setting the initial direction of control
 * and initializing the hysteresis and state values to default settings. This controller
 * is designed to manage systems that operate between two setpoints with a hysteresis band.
 *
 * @param direction The initial direction of control for the double setpoint controller.
 */
DoubleSetpointController::DoubleSetpointController(ControllerDirection direction)
    : Controller(direction)
{
  this->hysteresis = 2;
  this->low_state = 80;
  this->medium_state = 90;
  this->high_state = 100;
}

/**
 * @brief Destructor for the DoubleSetpointController class.
 *
 * Ensures that any resources allocated to the double setpoint controller are properly released.
 */
DoubleSetpointController::~DoubleSetpointController()
{
}

/**
 * @brief Sets the hysteresis value for the controller.
 *
 * Configures the hysteresis value, which determines the range around the setpoint where
 * the output remains unchanged, preventing oscillation or frequent toggling.
 *
 * @param hysteresis The hysteresis value, representing the allowable deviation from the setpoint.
 */
void DoubleSetpointController::setHysteresis(uint8_t hysteresis)
{
  if (!this->enabled)
    return;

  this->hysteresis = hysteresis;
}

/**
 * @brief Configures the output states for the controller.
 *
 * Sets the output values that correspond to the low, medium, and high states of the system
 * being controlled. These values are used to determine the controller's output based on the
 * current input and the setpoint with hysteresis.
 *
 * @param low_state The output value when the input is below the setpoint minus hysteresis.
 * @param medium_state The output value when the input is within the hysteresis range.
 * @param high_state The output value when the input is above the setpoint plus hysteresis.
 */
void DoubleSetpointController::setStates(int16_t low_state, int16_t medium_state, int16_t high_state)
{
  if (!this->enabled)
    return;

  this->low_state = low_state;
  this->medium_state = medium_state;
  this->high_state = high_state;
}

/**
 * @brief Retrieves the current hysteresis value of the controller.
 *
 * Returns the hysteresis value currently configured for the controller. This value
 * indicates the sensitivity of the controller to changes in the input around the setpoint.
 *
 * @return The hysteresis value used by the controller.
 */
uint8_t DoubleSetpointController::getHysteresis()
{
  if (!this->enabled)
    return 0;

  return this->hysteresis;
}

/**
 * @brief Computes and updates the controller's output based on the input.
 *
 * This function determines the appropriate output state of the controller by comparing
 * the input value to the setpoint and hysteresis. It adjusts the output to the low,
 * medium, or high state based on this comparison and the controller's direction.
 *
 * @param input The current input value from the system being controlled.
 * @return The updated output state of the controller.
 */
int16_t DoubleSetpointController::getOutput(int16_t input)
{
  if (!this->enabled)
    return 0;

  static unsigned long last_millis;
  this->input = input;

  if (millis() - last_millis > this->update_delay)
  {
    last_millis = millis();

    if (this->input >= this->setpoint - this->hysteresis && this->input <= this->setpoint + this->hysteresis)
    {
      this->output = this->medium_state;
    }
    else if (this->input < this->setpoint - this->hysteresis)
    {
      if (this->controller_direction == ControllerDirection::DIRECT)
      {
        this->output = this->high_state;
      }
      else if (this->controller_direction == ControllerDirection::REVERSE)
      {
        this->output = this->low_state;
      }
    }
    else if (this->input > this->setpoint + this->hysteresis)
    {
      if (this->controller_direction == ControllerDirection::DIRECT)
      {
        this->output = this->low_state;
      }
      else if (this->controller_direction == ControllerDirection::REVERSE)
      {
        this->output = this->high_state;
      }
    }
  }

  return this->output;
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @class  BangBangController
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Constructs a BangBangController object, inheriting from DoubleSetpointController.
 *
 * Initializes the BangBangController with a specified direction of control. This class
 * implements a bang-bang control strategy, which is a type of on-off control action where
 * the controller output is switched between two states without intermediate values.
 *
 * @param direction The initial direction of control (DIRECT or REVERSE).
 */
BangBangController::BangBangController(ControllerDirection direction)
    : DoubleSetpointController(direction)
{
}

/**
 * @brief Destructor for the BangBangController class.
 *
 * Cleans up the BangBangController object when it is no longer needed, ensuring that any
 * allocated resources are properly released.
 */
BangBangController::~BangBangController()
{
}

/**
 * @brief Sets the output states for the bang-bang controller.
 *
 * Configures the low and high output states of the controller. These states represent the
 * two possible outputs that the controller can switch between based on the input value and
 * the setpoint with hysteresis.
 *
 * @param low_state The output state when the input is below the setpoint minus hysteresis.
 * @param high_state The output state when the input is above the setpoint plus hysteresis.
 */
void BangBangController::setStates(int16_t low_state, int16_t high_state)
{
  if (!this->enabled)
    return;

  this->low_state = low_state;
  this->high_state = high_state;
}

/**
 * @brief Computes the output of the bang-bang controller based on the input.
 *
 * Determines the appropriate output state by comparing the input value to the setpoint
 * and hysteresis. The controller switches to the high state if the input is below the
 * setpoint minus hysteresis, and to the low state if the input is above the setpoint
 * plus hysteresis. The direction of control affects whether the high or low state is
 * used for each condition.
 *
 * @param input The current input value from the system being controlled.
 * @return The output state of the controller, either low_state or high_state.
 */
int16_t BangBangController::getOutput(int16_t input)
{
  if (!this->enabled)
    return 0;

  static unsigned long last_millis;
  this->input = input;

  if (millis() - last_millis > this->update_delay)
  {
    last_millis = millis();

    if (this->input < this->setpoint - this->hysteresis)
    {
      if (this->controller_direction == ControllerDirection::DIRECT)
      {
        this->output = this->high_state;
      }
      else if (this->controller_direction == ControllerDirection::REVERSE)
      {
        this->output = this->low_state;
      }
    }
    else if (this->input > this->setpoint + this->hysteresis)
    {
      if (this->controller_direction == ControllerDirection::DIRECT)
      {
        this->output = this->low_state;
      }
      else if (this->controller_direction == ControllerDirection::REVERSE)
      {
        this->output = this->high_state;
      }
    }
  }

  return this->output;
}