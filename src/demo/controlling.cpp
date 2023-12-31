/**
 * @file controller.cpp
 * @brief Implementation of the controller class.
<<<<<<< HEAD
 */
=======
*/
>>>>>>> 7156fcf (Large Update)

#include <api/Common.h>
#include <sys/_stdint.h>
#include "controlling.h"

<<<<<<< HEAD
=======

>>>>>>> 7156fcf (Large Update)
///----------------------------------------
/// @class CONTROLLER CLASS
///----------------------------------------

/**
<<<<<<< HEAD
 * @brief Constructor that initializes the controller direction.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
Controller::Controller(ControllerDirection direction)
    : direction_(direction), sample_time_(10)
{
}

/**
 * @brief Sets the desired setpoint for the controller.
 * @param setpoint_value The setpoint value.
 */
void Controller::setpoint(int16_t setpoint_value)
{
=======
  * @brief Constructor that initializes the controller direction.
  * @param direction The direction of the controller, either DIRECT or REVERSE.
*/
Controller::Controller(ControllerDirection direction)
  : direction_(direction), update_delay_(10) {
}

/**
  * @brief Sets the desired setpoint for the controller.
  * @param setpoint_value The setpoint value.
*/
void Controller::setpoint(int16_t setpoint_value) {
>>>>>>> 7156fcf (Large Update)
  setpoint_value_ = setpoint_value;
}

/**
<<<<<<< HEAD
 * @brief Sets the direction for the controller.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
void Controller::setControllerDirection(ControllerDirection direction)
{
=======
  * @brief Sets the direction for the controller.
  * @param direction The direction of the controller, either DIRECT or REVERSE.
*/
void Controller::setControllerDirection(ControllerDirection direction) {
>>>>>>> 7156fcf (Large Update)
  direction_ = direction;
}

/**
<<<<<<< HEAD
 * @brief Sets the sample time for the controller.
 * @param sample_time The sample time in milliseconds.
 */
void Controller::setSampleTime(uint8_t sample_time)
{
  sample_time_ = sample_time;
}

/**
 * @brief Plots the input, output and setpoint values on the serial monitor.
 */
void Controller::serialPlotGraph()
{
  if (Serial)
  {
=======
  * @brief Sets the sample time for the controller.
  * @param sample_time The sample time in milliseconds.
*/
void Controller::setSampleTime(uint8_t sample_time) {
  update_delay_ = sample_time;
}

/**
  * @brief Plots the input, output and setpoint values on the serial monitor.
*/
void Controller::serialPlotGraph() {
  if (Serial) {
>>>>>>> 7156fcf (Large Update)
    Serial.print(input_value_);
    Serial.print(",");
    Serial.print(setpoint_value_);
    Serial.print(",");
    Serial.println(output_value_);
  }
}

/**
<<<<<<< HEAD
 * @brief Returns the output value of the controller.
 * @return The output value.
 */
int16_t Controller::getOutput()
{
=======
  * @brief Returns the output value of the controller.
  * @return The output value.
*/
int16_t Controller::getOutput() {
>>>>>>> 7156fcf (Large Update)
  return output_value_;
}

///----------------------------------------
/// @class PID CONTROLLER CLASS
///----------------------------------------

/**
<<<<<<< HEAD
 * @brief Constructor that initializes the controller direction.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
PIDController::PIDController(ControllerDirection direction)
    : Controller(direction)
{                               // call the base class constructor
  this->tune(1.00, 1.00, 1.00); // set the default gains
  this->setLimits(0, 180);      // set the default limits
}

/**
 * @brief Updates the output value of the controller based on the input value.
 * @param input_value The input value of the system.
 */
void PIDController::update(int16_t input_value)
{
  static unsigned long last_ms; // use static variables to preserve the values between calls
=======
  * @brief Constructor that initializes the controller direction.
  * @param direction The direction of the controller, either DIRECT or REVERSE.
*/
PIDController::PIDController(ControllerDirection direction)
  : Controller(direction) {      // call the base class constructor
  this->tune(1.00, 1.00, 1.00);  // set the default gains
  this->setLimits(0, 180);       // set the default limits
}

/**
  * @brief Updates the output value of the controller based on the input value.
  * @param input_value The input value of the system.
*/
void PIDController::update(int16_t input_value) {
  static unsigned long last_ms;
>>>>>>> 7156fcf (Large Update)
  static int16_t last_error_value, error_sum;
  int16_t error_value, error_differential;
  input_value_ = input_value;

<<<<<<< HEAD
  if (millis() - last_ms > sample_time_)
  {
    error_value = setpoint_value_ - input_value_;
    error_sum += error_value * (millis() - last_ms);
    if (limits_configured_)
    {
=======
  if (millis() - last_ms > update_delay_) {
    error_value = setpoint_value_ - input_value_;
    error_sum += error_value * (millis() - last_ms);
    if (limits_configured_) {
>>>>>>> 7156fcf (Large Update)
      error_sum = constrain(error_sum, min_output_, max_output_);
    }
    error_differential = (error_value - last_error_value) / (millis() - last_ms);

    output_value_ = proportional_gain_ * error_value + integral_gain_ * error_sum + derivative_gain_ * error_differential;
<<<<<<< HEAD
    if (limits_configured_)
    {
=======
    if (limits_configured_) {
>>>>>>> 7156fcf (Large Update)
      limited_output_value_ = constrain(output_value_, min_output_, max_output_);
    }

    last_error_value = error_value;
    last_ms = millis();
  }
}

/**
<<<<<<< HEAD
 * @brief Tunes the PID parameters of the controller.
 * @param proportional_gain The proportional gain of the controller.
 * @param integral_gain The integral gain of the controller.
 * @param derivative_gain The derivative gain of the controller.
 */
void PIDController::tune(float proportional_gain, float integral_gain, float derivative_gain)
{
  if (proportional_gain < 0 || integral_gain < 0 || derivative_gain < 0)
  {
=======
  * @brief Tunes the PID parameters of the controller.
  * @param proportional_gain The proportional gain of the controller.
  * @param integral_gain The integral gain of the controller.
  * @param derivative_gain The derivative gain of the controller.
*/
void PIDController::tune(float proportional_gain, float integral_gain, float derivative_gain) {
  if (proportional_gain < 0 || integral_gain < 0 || derivative_gain < 0) {
>>>>>>> 7156fcf (Large Update)
    return;
  }
  proportional_gain_ = proportional_gain;
  integral_gain_ = integral_gain;
  derivative_gain_ = derivative_gain;

<<<<<<< HEAD
  if (direction_ == ControllerDirection::kReverse)
  {
=======
  if (direction_ == ControllerDirection::REVERSE) {
>>>>>>> 7156fcf (Large Update)
    proportional_gain_ = -proportional_gain_;
    integral_gain_ = -integral_gain_;
    derivative_gain_ = -derivative_gain_;
  }
}

/**
<<<<<<< HEAD
 * @brief Sets the output limits of the controller.
 * @param min_output The minimum output value of the controller.
 * @param max_output The maximum output value of the controller.
 */
void PIDController::setLimits(int16_t min_output, int16_t max_output)
{
  if (min_output < max_output)
  {
    min_output_ = min_output;
    max_output_ = max_output;
    limits_configured_ = true;
  }
  else
  {
=======
  * @brief Sets the output limits of the controller.
  * @param min_output The minimum output value of the controller.
  * @param max_output The maximum output value of the controller.
*/
void PIDController::setLimits(int16_t min_output, int16_t max_output) {
  if (min_output < max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
    limits_configured_ = true;
  } else {
>>>>>>> 7156fcf (Large Update)
    return;
  }
}

/**
<<<<<<< HEAD
 * @brief Returns the output value of the controller.
 * @return The output value.
 */
int16_t PIDController::getOutput()
{
  if (limits_configured_)
  {
    return limited_output_value_;
  }
  else
  {
=======
  * @brief Returns the output value of the controller.
  * @return The output value.
*/
int16_t PIDController::getOutput() {
  if (limits_configured_) {
    return limited_output_value_;
  } else {
>>>>>>> 7156fcf (Large Update)
    return output_value_;
  }
}

<<<<<<< HEAD
=======

>>>>>>> 7156fcf (Large Update)
///----------------------------------------
/// @class THREE STATE CONTROLLER CLASS
///----------------------------------------

/**
<<<<<<< HEAD
 * @brief Constructor that initializes the controller direction.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
ThreeStateController::ThreeStateController(ControllerDirection direction)
    : Controller(direction)
{                                       // call the base class constructor
  this->setHysteresis(2);               // set the default hysteresis
  this->setSteeringAngles(90, 80, 100); // set the default steering angles
}

/**
 * @brief Updates the output value of the controller based on the input value.
 * @param input_value The input value of the system.
 */
void ThreeStateController::update(int16_t input_value)
{
  static unsigned long last_ms; // use static variables to preserve the values between calls
  int16_t error_value;
  input_value_ = input_value;

  if (millis() - last_ms > sample_time_)
  {
    error_value = setpoint_value_ - input_value_;

    if (-hysteresis_ <= error_value && error_value <= hysteresis_)
    {
      output_value_ = straight_steering_angle_;
    }
    else
    {
      switch (direction_)
      {
      case ControllerDirection::kDirect:
      {
        if (error_value < -hysteresis_)
        {
          output_value_ = right_steering_angle_;
        }
        if (error_value > hysteresis_)
        {
          output_value_ = left_steering_angle_;
        }
      }
      break;
      case ControllerDirection::kReverse:
      {
        if (error_value < -hysteresis_)
        {
          output_value_ = left_steering_angle_;
        }
        if (error_value > hysteresis_)
        {
          output_value_ = right_steering_angle_;
        }
      }
      break;
=======
  * @brief Constructor that initializes the controller direction.
  * @param direction The direction of the controller, either DIRECT or REVERSE.
*/
ThreeStateController::ThreeStateController(ControllerDirection direction)
  : Controller(direction) {              // call the base class constructor
  this->setHysteresis(2);                // set the default hysteresis
  this->setSteeringAngles(90, 80, 100);  // set the default steering angles
}

/**
  * @brief Updates the output value of the controller based on the input value.
  * @param input_value The input value of the system.
*/
void ThreeStateController::update(int16_t input_value) {
  static unsigned long last_ms;
  int16_t error_value;
  input_value_ = input_value;

  if (millis() - last_ms > update_delay_) {
    error_value = setpoint_value_ - input_value_;

    if (-hysteresis_ <= error_value && error_value <= hysteresis_) {
      output_value_ = straight_steering_angle_;
    } else {
      switch (direction_) {
        case ControllerDirection::DIRECT:
          {
            if (error_value < -hysteresis_) {
              output_value_ = right_steering_angle_;
            }
            if (error_value > hysteresis_) {
              output_value_ = left_steering_angle_;
            }
          }
          break;
        case ControllerDirection::REVERSE:
          {
            if (error_value < -hysteresis_) {
              output_value_ = left_steering_angle_;
            }
            if (error_value > hysteresis_) {
              output_value_ = right_steering_angle_;
            }
          }
          break;
>>>>>>> 7156fcf (Large Update)
      }
    }

    last_ms = millis();
  }
}

/**
<<<<<<< HEAD
 * @brief Sets the hysteresis value of the controller.
 * @param hysteresis The hysteresis value in percentage (deviation from the error value).
 */
void ThreeStateController::setHysteresis(int8_t hysteresis)
{
=======
  * @brief Sets the hysteresis value of the controller.
  * @param hysteresis The hysteresis value in percentage (deviation from the error value).
*/
void ThreeStateController::setHysteresis(int8_t hysteresis) {
>>>>>>> 7156fcf (Large Update)
  hysteresis_ = hysteresis;
}

/**
<<<<<<< HEAD
 * @brief Sets the steering angles of the controller.
 * @param straight_steering_angle The steering angle for the straight direction in degrees.
 * @param left_steering_angle The steering angle for the left direction in degrees.
 * @param right_steering_angle The steering angle for the right direction in degrees.
 */
void ThreeStateController::setSteeringAngles(uint8_t straight_steering_angle, uint8_t left_steering_angle, uint8_t right_steering_angle)
{
=======
  * @brief Sets the steering angles of the controller.
  * @param straight_steering_angle The steering angle for the straight direction in degrees.
  * @param left_steering_angle The steering angle for the left direction in degrees.
  * @param right_steering_angle The steering angle for the right direction in degrees.
*/
void ThreeStateController::setSteeringAngles(uint8_t straight_steering_angle, uint8_t left_steering_angle, uint8_t right_steering_angle) {
>>>>>>> 7156fcf (Large Update)
  straight_steering_angle_ = straight_steering_angle;
  left_steering_angle_ = left_steering_angle;
  right_steering_angle_ = right_steering_angle;
}