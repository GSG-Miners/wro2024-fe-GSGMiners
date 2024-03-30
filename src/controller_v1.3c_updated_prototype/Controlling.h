/**
 * @file Controlling.h
 * @brief Header file for the DoubleSetpointController class.
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

enum class ControllerDirection : bool {
  DIRECT,
  REVERSE
};

class DoubleSetpointController {
public:
  DoubleSetpointController(ControllerDirection direction);
  ~DoubleSetpointController();

  void begin();
  void end();
  void setDirection(ControllerDirection direction);
  void setSetpoint(int16_t setpoint);
  void setStates(uint16_t low_state, uint16_t medium_state, uint16_t high_state);
  void setHysteresis(uint8_t hysteresis);
  int16_t getOutput(uint16_t input);

private:
  ControllerDirection direction;
  bool enabled;
  uint8_t hysteresis;
  int16_t setpoint;
  int16_t low_state;
  int16_t medium_state;
  int16_t high_state;
};

#endif