/**
 * @file controlling.h
 * @brief Controller class to get control a system that is defined
 by its given input, setpoint and other values that depend on the
 controller type (static controller / non-static controller).
 * @date 23rd December 2023 - 3rd February 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch, Finnian Belger & Logan Weigoldt
*/

#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <sys/_stdint.h>
#include <Arduino.h>

/**
 * @enum ControllerDirection
 * @brief This type defines whether the controller operates in direct or reverse mode.
 * @details In direct mode, the next error increases when the output increases.
 * In reverse mode, the next error decreases when the output increases.
 */

enum ControllerDirection : bool
{
  DIRECT = false,
  REVERSE = true
};

/**
 * @class Controller
 * @brief Abstract class for controlling a system with a given input and setpoint.
 */
class Controller
{
public:
  Controller(ControllerDirection direction);

  virtual void setpoint(int16_t setpoint_value);
  virtual void setControllerDirection(ControllerDirection direction);
  virtual void setSampleTime(uint8_t sample_time);
  virtual void serialPlotGraph();
  virtual int16_t getSetpoint();
  virtual int16_t getOutput();

protected:
  uint8_t update_delay_;          ///< The time interval between two updates in milliseconds
  int16_t input_value_;           ///< The input value of the system
  int16_t output_value_;          ///< The output value of the controller
  int16_t setpoint_value_;        ///< The desired setpoint value
  ControllerDirection direction_; ///< The direction of the controller
};

/**
 * @class PIDController
 * @brief Class for controlling a system with a PID algorithm.
 */
class PIDController : public Controller
{
public:
  PIDController(ControllerDirection direction);

  void update(int16_t input_value);
  void tune(float proportional_gain, float integral_gain, float derivative_gain);
  void setLimits(int16_t min_output, int16_t max_output);
  int16_t getOutput() override;

private:
  bool limits_configured_;       ///< Flag to indicate if the output limits are configured
  int16_t limited_output_value_; ///< The output value of the controller after applying the limits
  int16_t min_output_;           ///< The minimum output value of the controller
  int16_t max_output_;           ///< The maximum output value of the controller
  float proportional_gain_;      ///< The proportional gain of the controller
  float integral_gain_;          ///< The integral gain of the controller
  float derivative_gain_;        ///< The derivative gain of the controller
};

/**
 * @class ThreeStateController
 * @brief Class for controlling a system with a three-state algorithm.
 */
class ThreeStateController : public Controller
{
public:
  ThreeStateController(ControllerDirection direction);

  void update(int16_t input_value);
  void setHysteresis(int8_t hysteresis);
  void setSteeringAngles(uint8_t straight_steering_angle, uint8_t left_steering_angle, uint8_t right_steering_angle);
  int8_t getHysteresis();

private:
  uint8_t straight_steering_angle_; ///< The steering angle for the straight direction in degrees
  uint8_t left_steering_angle_;     ///< The steering angle for the left direction in degrees
  uint8_t right_steering_angle_;    ///< The steering angle for the right direction in degrees
  int8_t hysteresis_;               ///< The hysteresis value (deviation from the error value)
};

#endif // CONTROLLING_H