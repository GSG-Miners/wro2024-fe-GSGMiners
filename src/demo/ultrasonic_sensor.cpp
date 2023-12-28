/**
 * @file ultrasonic_sensor.cpp
 * @brief Implementation of the Ultrasonic sensor class.
*/

#include <api/Common.h>
#include <sys/_stdint.h>
#include "ultrasonic_sensor.h"

/**
 * @brief Ultrasonic sensor constructor.
 * @param trigger_pin The pin used to send out a signal.
 * @param echo_pin The Pin that produces a short pulse when the reflected signal
 * is received.
 * @param mode Determines, whether the measurements are started automatically (true) or
 * manually (false).
*/
UltrasonicSensor::UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin, Mode mode)
  : trigger_pin_(trigger_pin), echo_pin_(echo_pin), mode_(mode) {
  pinMode(trigger_pin_, OUTPUT);
  pinMode(echo_pin_, INPUT);
}

/**
 * @brief Ultrasonic sensor constructor.
 * @param trigger_pin The pin used to send out a signal.
 * @param echo_pin The Pin that produces a short pulse when the reflected signal
 * is received.
*/
UltrasonicSensor::UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin)
  : UltrasonicSensor(trigger_pin, echo_pin, Mode::kAutomatic) {}

/**
 * @brief Ultrasonic sensor destructur.
*/
UltrasonicSensor::~UltrasonicSensor() {}


/**
 * @brief Calculates the distance from measured pulse_time.
*/
void UltrasonicSensor::computeDistance() {
  unsigned long previous_us;

  digitalWrite(trigger_pin_, LOW);
  if (micros() - previous_us > 2) {
    previous_us = micros();
    digitalWrite(trigger_pin_, HIGH);
  }
  if (micros() - previous_us > 10) {
    previous_us = micros();
    digitalWrite(trigger_pin_, LOW);
  }
  unsigned long pulse_time = pulseInLong(echo_pin_, HIGH, 23292.37757);
  distance_ = pulse_time * kSpeedOfSound * pow(10, -4) / 2;
}

/**
 * @brief Updates the ultrasonic sensor's distance and status.
*/
void UltrasonicSensor::update() {
  if (mode_ == Mode::kAutomatic) {
    this->computeDistance();
  } else {
    if (status_) {
      this->computeDistance();
      status_ = false;
    }
  }
}

/**
 * @brief Sets the mode for the measurements.
 * @param mode Determines, whether the measurements are started automatically (true) or
 * manually (false).
*/
void UltrasonicSensor::setMode(Mode mode) {
  mode_ = mode;
}

/**
 * @brief Starts the measurement manually if mode is manual.
*/
void UltrasonicSensor::startMeasurement() {
  if (mode_ == Mode::kManual) {
    status_ = true;
  }
}

/**
 * @brief Checks if the ultrasonic sensor is updating the values.
 * @return True if the ultrasonic sensor is updating.
*/
bool UltrasonicSensor::isUpdating() {
  return status_;
}

/**
 * @brief Returns the the distance.
 * @return The distance to the object or wall.
*/
uint16_t UltrasonicSensor::getDistance() {
  distance_ = constrain(distance_, 0, 400);
  return distance_;
}