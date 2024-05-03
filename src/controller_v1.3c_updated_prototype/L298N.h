/**
 * @file L298N.h
 * @brief Header file for the L298N class, managing DC motor control via the L298N driver.
 *
 * This file declares the L298N class, which provides a high-level interface to control
 * DC motors using the L298N motor driver module. It simplifies the process of setting
 * motor speed and direction, making it accessible for projects that require motor
 * actuation.
 *
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef L298N_H
#define L298N_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#if defined(ARDUINO_ARCH_RENESAS)
#include "pwm.h"
#endif

#ifdef EXTENDED_PIN_MODE
typedef uint32_t pin_size_t;
#else
typedef uint8_t pin_size_t;
#endif

#define MIN_UPDATING_INTERVAL 50000
#define MAX_UPDATING_INTERVAL 100

class L298N
{
public:
  L298N(pin_size_t forward_pin, pin_size_t backward_pin);
  ~L298N();

  void begin();
  void end();
  void write(float speed);
  void stop();
  void runFor(unsigned long time, float speed);
  void setAcceleration(uint8_t acceleration);
  bool isUpdating();
  int8_t read();

private:
  PwmOut forwardPwm;
  PwmOut backwardPwm;
  bool enabled;
  bool is_updating;
  int8_t current_speed;
  int8_t setpoint_speed;
  uint8_t acceleration;
};

#endif