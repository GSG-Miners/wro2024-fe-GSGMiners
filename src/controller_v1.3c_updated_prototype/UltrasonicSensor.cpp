/**
 * @file UltrasonicSensor.cpp
 * @brief Implementation of the UltrasonicSensor class.
 */

#include "UltrasonicSensor.h"

/**
 * @brief Reads the pulse width of the ultrasonic sensor.
 *
 * Caches the port and bit of the pin in order to speed up the pulse width
 * measuring loop and achieve a finer resolution as digitalRead() yiels
 * a much coarser resolution.
 *
 * @param pin The pin connected to the echo signal of the sensor.
 * @param state The state to wait for (HIGH or LOW).
 * @param timeout The maximum time to wait for the pulse to start or end.
 * @return The pulse width in microseconds.
 */
uint32_t readPulseWidth(pin_size_t pin, bool state, float timeout) {
  pin_size_t port = digitalPinToPort(pin);
  uint16_t bit = digitalPinToBitMask(pin);
  uint16_t state_mask = state ? bit : 0;

  unsigned long process_start_micros = micros();

  while (((*portInputRegister(port)) & bit) == state_mask) {
    if (micros() - process_start_micros > timeout)
      return uint32_t(timeout);
  }

  while (((*portInputRegister(port)) & bit) != state_mask) {
    if (micros() - process_start_micros > timeout)
      return uint32_t(timeout);
  }

  unsigned long pulse_start_micros = micros();
  while (((*portInputRegister(port)) & bit) == state_mask) {
    if (micros() - process_start_micros > timeout)
      return uint32_t(timeout);
  }
  return micros() - pulse_start_micros;
}

/**
 * @brief Constructs an UltrasonicSensor object with a specified maximum distance.
 *
 * @param trigger_pin The pin connected to the trigger of the sensor.
 * @param echo_pin The pin connected to the echo of the sensor.
 * @param max_distance The maximum distance the sensor can measure.
 */
UltrasonicSensor::UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin, uint16_t max_distance)
  : trigger_pin(trigger_pin), echo_pin(echo_pin), max_distance(max_distance) {}

/**
 * @brief Constructs an UltrasonicSensor object with a default maximum distance.
 *
 * @param trigger_pin The pin connected to the trigger of the sensor.
 * @param echo_pin The pin connected to the echo of the sensor.
 */
UltrasonicSensor::UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin)
  : UltrasonicSensor(trigger_pin, echo_pin, 400) {}

/**
 * @brief Destructs the UltrasonicSensor object.
 */
UltrasonicSensor::~UltrasonicSensor() {}

/**
 * @brief Initializes the sensor.
 *
 * Sets the pinMode for trigger and echo pins, and begins the filter.
 */
void UltrasonicSensor::begin() {
  pinMode(this->trigger_pin, OUTPUT);
  pinMode(this->echo_pin, INPUT);
  this->filter.begin();
  this->enabled = true;
}

/**
 * @brief Disables the sensor.
 *
 * Ends the filter.
 */
void UltrasonicSensor::end() {
  this->filter.end();
  this->enabled = false;
}

/**
 * @brief Prints distance and average distance through serial communication.
 */
void UltrasonicSensor::print() {
  while (!Serial) {
  }

  Serial.print("Distance:");
  Serial.print(this->distance);
  Serial.print("\t");
  Serial.print("Average-Distance:");
  Serial.print(this->average_distance);
  Serial.print("\n");
}

/**
 * @brief Reads the distance from the sensor.
 *
 * @param temperature The current temperature for speed of sound calculation.
 * @return The distance in centimetres.
 */
uint16_t UltrasonicSensor::readDistance(int8_t temperature) {
  if (!this->enabled)
    return 0;

  if (this->computing_location != AVERAGE) {
    this->computing_location = DEFAULT;
  }

  static unsigned long last_micros;
  static uint8_t state;

  // Calculate speed of sound based on temperature
  float speed_of_sound = 331.3207951 + temperature * 0.6064814115;
  float timeout = 2 * max_distance * pow(10, 4) / speed_of_sound;

  // Send trigger pulse and measure pulse time
  digitalWrite(this->trigger_pin, LOW);
  if (micros() - last_micros > 2) {
    last_micros = micros();
    digitalWrite(this->trigger_pin, HIGH);
  }
  if (micros() - last_micros > 10) {
    last_micros = micros();
    digitalWrite(this->trigger_pin, LOW);
  }

  // Calculate the distance from the measured pulse time
  this->distance = readPulseWidth(this->echo_pin, HIGH, timeout) * speed_of_sound * pow(10, -4) / 2;

  return this->distance;
}

/**
 * @brief Reads the distance from the sensor at room temperature.
 *
 * @return The distance in centimetres.
 */
uint16_t UltrasonicSensor::readDistance() {
  this->readDistance(20);
}

/**
 * @brief Reads the average distance from the sensor.
 *
 * @param window_size The size of the moving average window.
 * @return The average in centimetres.
 */
uint16_t UltrasonicSensor::readAverageDistance(uint8_t window_size) {
  if (!this->enabled)
    return 0;

  if (this->computing_location != DEFAULT) {
    this->computing_location = AVERAGE;
    this->readDistance();
  }

  this->filter.add(this->distance);
  this->average_distance = this->filter.readAverage(window_size);

  return this->average_distance;
}

/**
 * @brief Checks if a peak is detected by the sensor.
 *
 * @param threshold_distance The threshold distance for peak detection.
 * @param consecutive_matches The number of consecutive matches required for peak detection.
 * @return True if a peak is detected, false otherwise.
 */
bool UltrasonicSensor::detectedPeak(uint16_t threshold_distance, uint8_t consecutive_matches) {
  if (!this->enabled)
    return 0;

  return filter.detectedPeak(threshold_distance, consecutive_matches);
}