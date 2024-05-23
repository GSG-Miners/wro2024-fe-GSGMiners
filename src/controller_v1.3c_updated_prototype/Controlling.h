/**
 * @file Controlling.h
 * @brief Header file for the Controller, PIDController, DoubleSetpointController, and BangBangController classes.
 *
 * This header file defines a comprehensive suite of control classes designed for a variety of control systems.
 * The base Controller class establishes a common interface and foundational methods for general control tasks.
 * The PIDController class provides a sophisticated Proportional-Integral-Derivative control mechanism, suitable
 * for systems requiring dynamic adjustments based on continuous feedback. The DoubleSetpointController class is
 * tailored for scenarios where control actions are determined by two distinct setpoints, offering a hysteresis
 * feature to prevent oscillation around the setpoint. The BangBangController class extends the
 * DoubleSetpointController with a simple yet effective on-off control strategy, ideal for applications where
 * precision is less critical, and a binary output is sufficient.
 *
 * The ControllerDirection enumeration simplifies the specification of control action direction, improving code
 * clarity and maintainability. These classes are versatile and can be integrated into diverse applications, from
 * straightforward home automation tasks to more complex industrial control systems, demonstrating adaptability
 * and scalability.
 *
 * By encapsulating control logic within these classes, the code structure remains clean and modular, aligning
 * with best practices in software design and facilitating ease of maintenance and future enhancements.
 *
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <sys/_stdint.h>
#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

enum class ControllerDirection : bool
{
  DIRECT,
  REVERSE
};

class Controller
{
public:
  Controller(ControllerDirection direction);
  ~Controller();

  virtual void begin();
  virtual void end();
  virtual void setDirection(ControllerDirection direction);
  virtual void setSetpoint(int16_t setpoint);
  virtual void setSampleTime(uint8_t sample_time);
  virtual void plotData();
  virtual int16_t getSetpoint();
  virtual int16_t getOutput(int16_t input);

protected:
  ControllerDirection controller_direction;
  bool enabled;
  uint8_t update_delay;
  int16_t input;
  int16_t output;
  int16_t setpoint;
};

class PIDController : public Controller
{
public:
  PIDController(ControllerDirection direction);
  ~PIDController();

  void tune(float proportional_gain, float integral_gain, float derivative_gain);
  void setLimits(int16_t min_output, int16_t max_output);
  int16_t getOutput(int16_t input) override;

private:
  bool limits_configured;
  int16_t constrained_output;
  int16_t min_output;
  int16_t max_output;
  float p_gain;
  float i_gain;
  float d_gain;
};

class DoubleSetpointController : public Controller
{
public:
  DoubleSetpointController(ControllerDirection direction);
  ~DoubleSetpointController();

  void setHysteresis(uint8_t hysteresis);
  void setStates(int16_t low_state, int16_t medium_state, int16_t high_state);
  uint8_t getHysteresis();
  virtual int16_t getOutput(int16_t input) override;

private:
  uint8_t hysteresis;
  int16_t low_state;
  int16_t medium_state;
  int16_t high_state;
};

class BangBangController : public DoubleSetpointController
{
public:
  BangBangController(ControllerDirection direction);
  ~BangBangController();

  void setStates(int16_t low_state, int16_t high_state);
  int16_t getOutput(int16_t input) override;

private:
  uint8_t hysteresis;
  int16_t low_state;
  int16_t high_state;
};

#endif // CONTROLLING_H