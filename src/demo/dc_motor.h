/**
 * @file dc_motor.h
 * @brief DC motor control class using the "pwm.h" module from the arduino-renesas core.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 25th December 2023 by Maximilian Kautzsch, 
 * Finnian Belger & Logan Weigoldt
*/

#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <sys/_stdint.h>
#include <Arduino.h>
#include "pwm.h"

// Constants
const int8_t kMinSpeed = -100;
const uint8_t kMaxSpeed = 100;
const uint8_t kMinAcceleration = 0;
const uint8_t kMaxAcceleration = 100;
const uint8_t kMinSpeedUpdateInterval = 50;
const uint8_t kMaxSpeedUpdateInterval = 0;
const uint8_t kInitialDutyCycle = 0;
const uint16_t kPwmFrequency = 33000;

/**
 * @class Motor
 * @brief Class for controlling a DC motor.
*/
class Motor {
public:
  Motor(uint8_t forward_pin, uint8_t backward_pin);
  ~Motor();

  bool init();
  void update();
  void setSpeed(int8_t speed);
  void setAcceleration(uint8_t acceleration);
  bool stop();
  bool isUpdating();
  int8_t getSpeed();

private:
  PwmOut forwardPwm;
  PwmOut backwardPwm;
  bool enabled_;
  bool status_;
  int8_t current_speed_;
  int8_t setpoint_speed_;
  uint8_t acceleration_;
  uint8_t speed_update_interval_;
  uint16_t frequency_;
  unsigned long last_speed_update_time_;
};

#endif  // DC_MOTOR_H