/**
 * @file pid_controller.h
 * @brief PID controller class to get control a system that is defined
 by its given input, setpoint and controller gain values.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 25th December 2023 by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
*/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <sys/_stdint.h>
#include <Arduino.h>

enum ControllerDirection : bool {
  kDirect = false,
  kReverse = true,
};

class PIDController {
public:
  PIDController(ControllerDirection direction);

  void update(int16_t input_value);
  void tune(float proportional_gain, float integral_gain, float derivative_gain);
  void setpoint(int16_t setpoint_value);
  void setControllerDirection(ControllerDirection direction);
  void setLimits(int16_t min_output, int16_t max_output);
  void setSampleTime(uint8_t sample_time);
  void serialPlotGraph();
  int16_t getOutput();

private:
  bool limits_configured_;
  uint8_t sample_time_;
  int16_t input_value_;
  int16_t output_value_;
  int16_t min_output_;
  int16_t max_output_;
  int16_t setpoint_value_;
  float proportional_gain_;
  float integral_gain_;
  float derivative_gain_;
  ControllerDirection direction_;
};

#endif