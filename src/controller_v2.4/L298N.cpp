/**
 * @file L298N.cpp
 * @brief Implementation of the L298N class.
 */

#include "L298N.h"

/**
 * @brief Constructor for the L298N class.
 *
 * Initializes the motor driver by assigning the provided pins to the forward and backward
 * PWM control variables. These pins will be used to control the direction and speed of the
 * connected DC motor.
 *
 * @param forward_pin The pin number assigned to control the forward motion of the motor.
 * @param backward_pin The pin number assigned to control the backward motion of the motor.
 */
L298N::L298N(pin_size_t forward_pin, pin_size_t backward_pin)
  : forwardPwm(forward_pin), backwardPwm(backward_pin) {}

/**
 * @brief Destructor for the L298N class.
 *
 * Ensures that any resources allocated to the motor driver are properly released. This is
 * particularly important in environments where resource management is critical.
 */
L298N::~L298N() {}

/**
 * @brief Initializes the motor driver for operation.
 *
 * Sets up the PWM frequency for both forward and backward control pins and enables the motor
 * driver for use. The acceleration is set to a default value, and the motor driver is marked
 * as enabled.
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
 * Ends the PWM signals on both control pins, effectively stopping the motor and marking the
 * driver as disabled. This method should be called when the motor is no longer needed to
 * ensure it does not continue to run.
 */
void L298N::end() {
  forwardPwm.end();
  backwardPwm.end();
  this->enabled = false;
}

/**
 * @brief Sets the motor speed to a specified value.
 *
 * Updates the motor's speed based on the desired input speed, ranging from -100 to 100. The
 * speed is adjusted gradually based on the set acceleration to provide a smooth transition.
 * Negative values reverse the motor's direction, while positive values move it forward.
 *
 * @param speed The target speed for the motor, where -100 is full reverse, 0 is stopped, and
 * 100 is full forward.
 */
void L298N::write(float speed) {
  if (!this->enabled)
    return;

  static unsigned long last_micros;
  uint16_t updating_interval = map(this->acceleration, 0, 100, MIN_UPDATING_INTERVAL, MAX_UPDATING_INTERVAL);
  this->setpoint_speed = constrain(speed, -100, 100);

  // Adjust the current speed to the new setSetpoint speed
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
 * @brief Immediately stops the motor.
 *
 * Sets both the current and target speeds to zero and sends a corresponding PWM signal to
 * stop the motor instantly. This method is useful for emergency stops or when a quick response
 * is required.
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
 * @brief Operates the motor for a specified duration at a given speed.
 *
 * Starts the motor at the desired speed and allows it to run for the specified duration. If
 * the duration elapses, the motor is automatically stopped. This method is useful for tasks
 * that require the motor to run for a predetermined amount of time.
 *
 * @param duration The time in milliseconds for which the motor should run.
 * @param speed The speed at which the motor should operate.
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
 * @brief Sets the rate of change of the motor's speed.
 *
 * Defines how quickly the motor can change its speed, with a range from 0 (instantaneous
 * change) to 100 (slowest change). This allows for smooth acceleration and deceleration,
 * which can be crucial for applications that require precise speed control.
 *
 * @param acceleration The desired acceleration rate.
 */
void L298N::setAcceleration(uint8_t acceleration) {
  if (!this->enabled)
    return;

  this->acceleration = constrain(acceleration, 0, 100);
}

/**
 * @brief Indicates whether the motor's speed is in the process of changing.
 *
 * Checks if the motor's speed is currently being updated towards the target speed. This
 * can be used to monitor the progress of speed changes or to synchronize other actions
 * with the motor's operation.
 *
 * @return True if the motor's speed is updating, false if it has reached the target speed.
 */
bool L298N::isUpdating() {
  if (!this->enabled)
    return 0;

  return this->is_updating;
}

/**
 * @brief Retrieves the current operating speed of the motor.
 *
 * Returns the current speed at which the motor is running. This can be used to monitor
 * the motor's status or to make decisions based on its current speed.
 *
 * @return The current speed of the motor, ranging from -100 to 100.
 */
int8_t L298N::read() {
  if (!this->enabled)
    return 0;

  return this->current_speed;
}