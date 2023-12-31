/**
 * @file ultrasonic_sensor.h
 * @brief Ultrasonic sensor class for managing the distance of a ultrasonic sensor.
 * @date 23rd December 2023 - 31st January 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch, Finnian Belger & Logan Weigoldt
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <sys/_stdint.h>
#include <Arduino.h>

// Constants
const float kSpeedOfSound = 343.46;

/**
 * @enum SonarMode
 * @brief Defines whether the ultrasonic sensor operates in manual or automatic mode.
 * @details In manual mode, the measurement is triggered by calling the startMeasurement() method.
 * In automatic mode, the measurement is triggered by calling the update() method.
 */
enum SonarMode : const bool
{
  kManual = false,
  kAutomatic = true
};

/**
 * @class UltrasonicSensor
 * @brief Class for managing the distance returned by the ultrasonic sensor.
 */
class UltrasonicSensor
{
public:
  UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin, SonarMode mode);
  UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin);
  ~UltrasonicSensor();

  void update();
  void setMode(SonarMode mode);
  void startMeasurement();
  bool isUpdating();
  uint16_t getDistance();

private:
  void computeDistance(); ///< The method to calculate the distance from the echo signal duration using the speed of sound
  SonarMode mode_;        ///< The mode of the ultrasonic sensor, either MANUAL or AUTOMATIC
  bool status_;           ///< The status of the ultrasonic sensor, true if it is updating, false otherwise
  uint8_t trigger_pin_;   ///< The pin number for the trigger signal of the ultrasonic sensor
  uint8_t echo_pin_;      ///< The pin number for the echo signal of the ultrasonic sensor
  uint16_t distance_;     ///< The distance measured by the ultrasonic sensor in centimeters
};

#endif // ULTRASONIC_SENSOR_H