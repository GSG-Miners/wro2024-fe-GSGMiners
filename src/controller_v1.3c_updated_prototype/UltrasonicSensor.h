/**
 * @file UltrasonicSensor.h
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <inttypes.h>
#include "MovingAverage.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef EXTENDED_PIN_MODE
typedef uint32_t pin_size_t;
#else
typedef uint8_t pin_size_t;
#endif

#define DEFAULT 1
#define AVERAGE 2

class UltrasonicSensor {
public:
  UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin, uint16_t max_distance);
  UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin);
  ~UltrasonicSensor();

  void begin();
  void end();
  void print();
  bool detectedPeak(uint16_t threshold_distance, uint8_t consecutive_matches);
  uint16_t readDistance(int8_t temperature);
  uint16_t readDistance();
  uint16_t readAverageDistance(uint8_t window_size);

private:
  MovingAverage<uint16_t, uint16_t> filter;
  bool enabled;
  pin_size_t trigger_pin;
  pin_size_t echo_pin;
  uint8_t computing_location;
  uint16_t distance;
  uint16_t average_distance;
  uint16_t max_distance;
};

#endif