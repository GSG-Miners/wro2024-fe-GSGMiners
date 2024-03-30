/**
 * @file L298N.cpp
 * @brief Implementation of the L298N class.
 */

#include "L298N.h"

/**
 * @brief Constructor for L298N class.
 * @param forward_pin Pin connected to the forward control of the motor driver.
 * @param backward_pin Pin connected to the backward control of the motor driver.
 */
L298N::L298N(pin_size_t forward_pin, pin_size_t backward_pin)
  : forwardPwm(forward_pin), backwardPwm(backward_pin) {}

/**
 * @brief Destructor for L298N class.
 */
L298N::~L298N() {}

/**
 * @brief Initializes the motor driver.
 *
 * Sets the PWM frequency and initializes the PWM outputs.
 */
void L298N::begin() {
  this->acceleration = 100;
  forwardPwm.begin(float(33000.00), 0.00);
  backwardPwm.begin(float(33000.00), 0.00);
  this->enabled = true;
}

/**
 * @brief Disables the motor driver.
 *
 * Ends the PWM outputs.
 */
void L298N::end() {
  forwardPwm.end();
  backwardPwm.end();
  this->enabled = false;
}

/**
 * @brief Sets the motor speed.
 * @param speed Desired speed (-100 to 100).
 *
 * Updates the motor speed based on the set acceleration and updating interval.
 */
void L298N::write(float speed) {
  if (!this->enabled)
    return;

  static unsigned long last_micros;
  uint16_t updating_interval = map(this->acceleration, 0, 100, MIN_UPDATING_INTERVAL, MAX_UPDATING_INTERVAL);
  this->setpoint_speed = constrain(speed, -100, 100);

  // Adjust the current speed to the new setpoint speed
  if (micros() - last_micros > updating_interval) {
    last_micros = micros();

    if (this->setpoint_speed > this->current_speed) {
      this->current_speed++;
      this->is_updating = true;
    } else if (this->setpoint_speed < this->current_speed) {
      this->current_speed--;
      this->is_updating = true;
    } else {
      this->is_updating = false;
    }
  }

  // Transfer the updated speed to the pwm pins
  if (this->current_speed < 0) {
    forwardPwm.pulse_perc(0);
    backwardPwm.pulse_perc(abs(this->current_speed));
  } else {
    forwardPwm.pulse_perc(this->current_speed);
    backwardPwm.pulse_perc(0);
  }
}

/**
 * @brief Stops the motor immediately.
 *
 * Sets the current and setpoint speeds to zero and updates the PWM outputs.
 */
void L298N::stop() {
  if (!this->enabled)
    return;

  this->current_speed = 0;
  this->setpoint_speed = 0;
  forwardPwm.pulse_perc(this->current_speed);
  backwardPwm.pulse_perc(this->current_speed);
}

/**
 * @brief Runs the motor for a specified duration at a given speed.
 * 
 * This function starts the motor at the specified speed and runs it for the given amount of time.
 * If the motor is not enabled, the function will return immediately.
 * The function uses a static variable to keep track of the last time the motor was started.
 * If the current time exceeds the specified duration, the motor is stopped.
 *
 * @param duration The duration for which the motor should run (in milliseconds).
 * @param speed The speed at which the motor should run (-100 to 100).
 */
void L298N::runFor(unsigned long duration, float speed) {
  if (!this->enabled)
    return;

  static unsigned long last_millis;
  
  this->write(speed);

  if (millis() - last_millis > duration) {
    this->end();
  }
}

/**
 * @brief Sets the acceleration for the motor.
 * @param acceleration Desired acceleration (0 to 100).
 *
 * Constrains the acceleration value and updates the internal variable.
 */
void L298N::setAcceleration(uint8_t acceleration) {
  if (!this->enabled)
    return;

  this->acceleration = constrain(acceleration, 0, 100);
}

/**
 * @brief Checks if the motor speed is currently updating.
 * @return True if updating, false otherwise.
 */
bool L298N::isUpdating() {
  if (!this->enabled)
    return 0;

  return this->is_updating;
}

/**
 * @brief Reads the current motor speed.
 * @return Current speed (-100 to 100).
 */
int8_t L298N::read() {
  if (!this->enabled)
    return 0;

  return this->current_speed;
}