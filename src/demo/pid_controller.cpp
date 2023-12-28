/**
 * @file pid_controller.cpp
 * @brief Implementation of the PID controller class.
*/

#include <api/Common.h>
#include <sys/_stdint.h>
#include "pid_controller.h"

/**
 * @brief PID controller constructor.
*/
PIDController::PIDController(ControllerDirection direction)
  : direction_(direction), sample_time_(50),
    proportional_gain_(1.00), integral_gain_(1.00), derivative_gain_(1.00) {
  this->setLimits(0, 180);
}

/**
 * @brief Computes the output value out of input and setpoint value using the PID controlling algorithm.
 * @param input_value The sensor input from which, together with the setpoint, the output_value is
 * computed.
*/
void PIDController::update(int16_t input_value) {
  unsigned long last_ms;
  uint16_t error_value, last_error_value, error_sum, error_differential;
  input_value_ = input_value;

  if (millis() - last_ms > sample_time_) {
    error_value = setpoint_value_ - input_value_;
    error_sum += error_value * (millis() - last_ms);
    if (limits_configured_) {
      error_sum = constrain(error_sum, min_output_, max_output_);
    }
    error_differential = (error_value - last_error_value) / (millis() - last_ms);

    output_value_ = proportional_gain_ * error_value + integral_gain_ * error_sum + derivative_gain_ * error_differential;
    if (limits_configured_) {
      output_value_ = constrain(output_value_, min_output_, max_output_);
    }

    last_error_value = error_value;
    last_ms = millis();
  }
}

/**
 * @brief Sets the gains of the PID controller.
 * @param proportional_gain The gain of the proportional term.
 * @param integral_gain The gain of the integral term.
 * @param derivative_gain The gain of the derivative term.
*/
void PIDController::tune(float proportional_gain, float integral_gain, float derivative_gain) {
  if (proportional_gain < 0 || integral_gain < 0 || derivative_gain < 0) {
    return;
  }
  proportional_gain_ = proportional_gain;
  integral_gain_ = integral_gain;
  derivative_gain_ = derivative_gain;
  
  if (direction_ == ControllerDirection::kReverse) {
    proportional_gain_ = -proportional_gain_;
    integral_gain_ = -integral_gain_;
    derivative_gain_ = -derivative_gain_;
  }
}

/**
 * @brief Configures the desired process variable aka setpoint value.
 * @param setpoint_value The setpoint value.
*/
void PIDController::setpoint(int16_t setpoint_value) {
  setpoint_value_ = setpoint_value;
}

/**
 * @brief Sets the controller direction.
 * @param direction The direction of the controller (direct or reverse).
*/

void PIDController::setControllerDirection(ControllerDirection direction) {
  direction_ = direction;
}

/**
 * @brief Constrains the output so that it lies in the range [min_output, max_output]
 * @param min_output The maximum output value.
 * @param max_output The minimum output value.
*/
void PIDController::setLimits(int16_t min_output, int16_t max_output) {
  if (min_output < max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
    limits_configured_ = true;
  } else {
    return;
  }
}

/**
 * @brief Sets the period at which the calculation is performed.
 * @param sample_time The time interval in milliseconds.
*/
void PIDController::setSampleTime(uint8_t sample_time) {
  sample_time_ = sample_time;
}

/**
 * @brief Plots the input, output and setpoint value through the serial plotter.
*/
void PIDController::serialPlotGraph() {
  if (Serial) {
    Serial.print(input_value_);
    Serial.print(",");
    Serial.print(setpoint_value_);
    Serial.print(",");
    Serial.println(output_value_);
  }
}

/**
 * @brief Returns the output value that is calculated using the update() method.
 * @return The controller output.
*/
int16_t PIDController::getOutput() {
  return output_value_;
}