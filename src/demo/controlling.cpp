/**
 * @file controller.cpp
 * @brief Implementation of the controller class.
 */

#include <api/Common.h>
#include <sys/_stdint.h>
#include "controlling.h"

///----------------------------------------
/// @class CONTROLLER CLASS
///----------------------------------------

/**
 * @brief Constructor that initializes the controller direction.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
Controller::Controller(ControllerDirection direction)
    : direction_(direction), update_delay_(10)
{
}

/**
 * @brief Sets the desired setpoint for the controller.
 * @param setpoint_value The setpoint value.
 */
void Controller::setpoint(int16_t setpoint_value)
{
  setpoint_value_ = setpoint_value;
}

/**
 * @brief Sets the direction for the controller.
 * @param direction The direction of the controller, either DIRECT or REVERSE.
 */
void Controller::setControllerDirection(ControllerDirection direction)
{
  direction_ = direction;
}

/**
 * @brief Sets the sample time for the controller.
 * @param sample_time The sample time in milliseconds.
 */
void Controller::setSampleTime(uint8_t sample_time)
{
  update_delay_ = sample_time;
}

/**
 * @brief Plots the input, output and setpoint values on the serial monitor.
 */
void Controller::serialPlotGraph()
{
  if (Serial)
  {
    Serial.print(input_value_);
    Serial.print(",");
    Serial.print(setpoint_value_);
    Serial.print(",");
    Serial.println(output_value_);
  }
}

/**
 * @brief Returns the setpoint value of the controller.
 * @return The setpoint value.
 */
int16_t Controller::getSetpoint()
{
  return setpoint_value_;
}

/**
 * @brief Returns the output value of the controller.
 * @return The output value.
 */
int16_t Controller::getOutput()
{
  return output_value_;
}

///----------------------------------------
/// @class PID CONTROLLER CLASS
///----------------------------------------

/**
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
  static unsigned long last_ms;
  static int16_t last_error_value, error_sum;
  int16_t error_value, error_differential;
  input_value_ = input_value;

  if (millis() - last_ms > update_delay_)
  {
    error_value = setpoint_value_ - input_value_;
    error_sum += error_value * (millis() - last_ms);
    if (limits_configured_)
    {
      error_sum = constrain(error_sum, min_output_, max_output_);
    }
    error_differential = (error_value - last_error_value) / (millis() - last_ms);

    output_value_ = proportional_gain_ * error_value + integral_gain_ * error_sum + derivative_gain_ * error_differential;
    if (limits_configured_)
    {
      limited_output_value_ = constrain(output_value_, min_output_, max_output_);
    }

    last_error_value = error_value;
    last_ms = millis();
  }
}

/**
 * @brief Tunes the PID parameters of the controller.
 * @param proportional_gain The proportional gain of the controller.
 * @param integral_gain The integral gain of the controller.
 * @param derivative_gain The derivative gain of the controller.
 */
void PIDController::tune(float proportional_gain, float integral_gain, float derivative_gain)
{
  if (proportional_gain < 0 || integral_gain < 0 || derivative_gain < 0)
  {
    return;
  }
  proportional_gain_ = proportional_gain;
  integral_gain_ = integral_gain;
  derivative_gain_ = derivative_gain;

  if (direction_ == ControllerDirection::REVERSE)
  {
    proportional_gain_ = -proportional_gain_;
    integral_gain_ = -integral_gain_;
    derivative_gain_ = -derivative_gain_;
  }
}

/**
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
    return;
  }
}

/**
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
    return output_value_;
  }
}

///----------------------------------------
/// @class THREE STATE CONTROLLER CLASS
///----------------------------------------

/**
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
  static unsigned long last_ms;
  int16_t error_value;
  input_value_ = input_value;

  if (millis() - last_ms > update_delay_)
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
      case ControllerDirection::DIRECT:
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
      case ControllerDirection::REVERSE:
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
      }
    }

    last_ms = millis();
  }
}

/**
 * @brief Sets the hysteresis value of the controller.
 * @param hysteresis The hysteresis value in percentage (deviation from the error value).
 */
void ThreeStateController::setHysteresis(int8_t hysteresis)
{
  hysteresis_ = hysteresis;
}

/**
 * @brief Sets the steering angles of the controller.
 * @param straight_steering_angle The steering angle for the straight direction in degrees.
 * @param left_steering_angle The steering angle for the left direction in degrees.
 * @param right_steering_angle The steering angle for the right direction in degrees.
 */
void ThreeStateController::setSteeringAngles(uint8_t straight_steering_angle, uint8_t left_steering_angle, uint8_t right_steering_angle)
{
  straight_steering_angle_ = straight_steering_angle;
  left_steering_angle_ = left_steering_angle;
  right_steering_angle_ = right_steering_angle;
}

/**
 * @brief Retuns the current hysteresis value of the controller.
 * @return The hysteresis value.
 */
int8_t ThreeStateController::getHysteresis()
{
  return hysteresis_;
}