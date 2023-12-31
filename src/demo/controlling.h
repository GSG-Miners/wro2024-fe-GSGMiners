/**
 * @file controlling.h
 * @brief Controller class to get control a system that is defined
 by its given input, setpoint and other values that depend on the
 controller type (static controller / non-static controller).
<<<<<<< HEAD
 * @date 23rd December 2023 - 31st January 2024
=======
 * @date 23rd December 2023 - 3rd February 2024
>>>>>>> 7156fcf (Large Update)
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
<<<<<<< HEAD
 */

enum ControllerDirection : bool
{
  kDirect = false,
  kReverse = true,
=======
*/

enum ControllerDirection : bool {
  DIRECT = false,
  REVERSE = true
>>>>>>> 7156fcf (Large Update)
};

/**
 * @class Controller
 * @brief Abstract class for controlling a system with a given input and setpoint.
<<<<<<< HEAD
 */
class Controller
{
=======
*/
class Controller {
>>>>>>> 7156fcf (Large Update)
public:
  Controller(ControllerDirection direction);

  virtual void setpoint(int16_t setpoint_value);
  virtual void setControllerDirection(ControllerDirection direction);
  virtual void setSampleTime(uint8_t sample_time);
  virtual void serialPlotGraph();
  virtual int16_t getOutput();

protected:
<<<<<<< HEAD
  uint8_t sample_time_;           ///< The sample time in milliseconds
  int16_t input_value_;           ///< The input value of the system
  int16_t output_value_;          ///< The output value of the controller
  int16_t setpoint_value_;        ///< The desired setpoint value
  ControllerDirection direction_; ///< The direction of the controller
=======
  uint8_t update_delay_;           ///< The time interval between two updates in milliseconds
  int16_t input_value_;            ///< The input value of the system
  int16_t output_value_;           ///< The output value of the controller
  int16_t setpoint_value_;         ///< The desired setpoint value
  ControllerDirection direction_;  ///< The direction of the controller
>>>>>>> 7156fcf (Large Update)
};

/**
 * @class PIDController
 * @brief Class for controlling a system with a PID algorithm.
<<<<<<< HEAD
 */
class PIDController : public Controller
{
=======
*/
class PIDController : public Controller {
>>>>>>> 7156fcf (Large Update)
public:
  PIDController(ControllerDirection direction);

  void update(int16_t input_value);
  void tune(float proportional_gain, float integral_gain, float derivative_gain);
  void setLimits(int16_t min_output, int16_t max_output);
  int16_t getOutput() override;

private:
<<<<<<< HEAD
  bool limits_configured_;       ///< Flag to indicate if the output limits are configured
  int16_t limited_output_value_; ///< The output value of the controller after applying the limits
  int16_t min_output_;           ///< The minimum output value of the controller
  int16_t max_output_;           ///< The maximum output value of the controller
  float proportional_gain_;      ///< The proportional gain of the controller
  float integral_gain_;          ///< The integral gain of the controller
  float derivative_gain_;        ///< The derivative gain of the controller
=======
  bool limits_configured_;        ///< Flag to indicate if the output limits are configured
  int16_t limited_output_value_;  ///< The output value of the controller after applying the limits
  int16_t min_output_;            ///< The minimum output value of the controller
  int16_t max_output_;            ///< The maximum output value of the controller
  float proportional_gain_;       ///< The proportional gain of the controller
  float integral_gain_;           ///< The integral gain of the controller
  float derivative_gain_;         ///< The derivative gain of the controller
>>>>>>> 7156fcf (Large Update)
};

/**
 * @class ThreeStateController
 * @brief Class for controlling a system with a three-state algorithm.
<<<<<<< HEAD
 */
class ThreeStateController : public Controller
{
=======
*/
class ThreeStateController : public Controller {
>>>>>>> 7156fcf (Large Update)
public:
  ThreeStateController(ControllerDirection direction);

  void update(int16_t input_value);
  void setHysteresis(int8_t hysteresis);
  void setSteeringAngles(uint8_t straight_steering_angle, uint8_t left_steering_angle, uint8_t right_steering_angle);

private:
<<<<<<< HEAD
  uint8_t straight_steering_angle_; ///< The steering angle for the straight direction in degrees
  uint8_t left_steering_angle_;     ///< The steering angle for the left direction in degrees
  uint8_t right_steering_angle_;    ///< The steering angle for the right direction in degrees
  int8_t hysteresis_;               ///< The hysteresis value (deviation from the error value)
};

#endif // CONTROLLING_H
=======
  uint8_t straight_steering_angle_;  ///< The steering angle for the straight direction in degrees
  uint8_t left_steering_angle_;      ///< The steering angle for the left direction in degrees
  uint8_t right_steering_angle_;     ///< The steering angle for the right direction in degrees
  int8_t hysteresis_;                ///< The hysteresis value (deviation from the error value)
};

#endif  // CONTROLLING_H
>>>>>>> 7156fcf (Large Update)
