/**
 * @file dc_motor.cpp
 * @brief Implementation of the Motor class.
*/

#include <api/Common.h>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include "dc_motor.h"

/**
 * @brief Motor constructor.
*/
Motor::Motor(uint8_t forward_pin, uint8_t backward_pin)
  : forwardPwm(forward_pin), backwardPwm(backward_pin) {}

/**
 * @brief Motor destructor.
*/
Motor::~Motor() {
  forwardPwm.end();
  backwardPwm.end();
  enabled_ = false;
}

/**
 * @brief Initializes the motor parameters and PwmOut objects.
 * @return True if initialization was successful.
*/
bool Motor::init() {
  acceleration_ = kMaxAcceleration;
  frequency_ = kPwmFrequency;
  forwardPwm.begin(frequency_, kInitialDutyCycle);
  backwardPwm.begin(frequency_, kInitialDutyCycle);
  enabled_ = true;
  return enabled_;
}

/**
 * @brief Updates the parameters and adjusts the current speed
 * to the setpoint speed in a given time interval.
*/
void Motor::update() {
  speed_update_interval_ = map(acceleration_, kMinAcceleration, kMaxAcceleration,
                               kMinSpeedUpdateInterval, kMaxSpeedUpdateInterval);

  // adjust the current speed to the setpoint speed in time interval
  if (millis() > speed_update_interval_ + last_speed_update_time_) {
    last_speed_update_time_ = millis();
    if (setpoint_speed_ != current_speed_) {
      status_ = true;
      if (setpoint_speed_ > current_speed_) {
        current_speed_++;
      } else if (setpoint_speed_ < current_speed_) {
        current_speed_--;
      }
    } else {
      status_ = false;
    }

    // change the duty cycle of the pins, depending on the direction
    if (current_speed_ < 0) {
      forwardPwm.pulse_perc(0);
      backwardPwm.pulse_perc(abs(current_speed_));
    } else {
      forwardPwm.pulse_perc(current_speed_);
      backwardPwm.pulse_perc(0);
    }
  }
}

/**
 * @brief Sets the desired speed of the DC motor in percent.
 * If the speed is positive, the motor will drive forward, otherwise
 * it will drive backward. The range of the speed is between
 * [-100; 100].
 * @param speed The desired speed.
*/
void Motor::setSpeed(int8_t speed) {
  setpoint_speed_ = constrain(speed, kMinSpeed, kMaxSpeed);
}

/**
 * @brief Sets the acceleration of the DC motor in percent.
 * Therefore, the range for the acceleration is [0; 100]. 
 * @param acceleration The desired acceleration.
*/
void Motor::setAcceleration(uint8_t acceleration) {
  acceleration_ = constrain(acceleration, kMinAcceleration, kMaxAcceleration);
}

/**
 * @brief Stops the DC motor.
 * @return False as the motor got stopped.
*/
bool Motor::stop() {
  // immediately set both current speed and setpoint speed to 0
  acceleration_ = kMaxAcceleration;
  current_speed_ = 0;
  setpoint_speed_ = 0;
  enabled_ = false;
  return enabled_;
}

/**
 * @brief Checks if the DC motor is updating.
 * @return True if the DC motor is updating.
*/
bool Motor::isUpdating() {
  return status_;
}

/**
 * @brief Returns the current speed of the DC motor.
 * @return The current speed.
*/
int8_t Motor::getSpeed() {
  return current_speed_;
}