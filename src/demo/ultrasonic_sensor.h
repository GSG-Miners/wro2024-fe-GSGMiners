/**
 * @file ultrasonic_sensor.h
 * @brief Ultrasonic sensor class for managing the distance of a ultrasonic sensor.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 25th December 2023 by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
*/

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <sys/_stdint.h>
#include <Arduino.h>

// Constants
const float kSpeedOfSound = 343.46;

enum Mode : const bool {
  kManual = false,
  kAutomatic = true,
};

/**
 * @class UltrasonicSensor
 * @brief Class to for managing the distance returned by the ultrasonic sensor.
*/
class UltrasonicSensor {
public:
  UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin, Mode mode);
  UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin);
  ~UltrasonicSensor();

  void update();
  void setMode(Mode mode);
  void startMeasurement();
  bool isUpdating();
  uint16_t getDistance();

private:
  void computeDistance();
  Mode mode_;
  bool status_;
  uint8_t trigger_pin_;
  uint8_t echo_pin_;
  uint16_t distance_;
};

#endif