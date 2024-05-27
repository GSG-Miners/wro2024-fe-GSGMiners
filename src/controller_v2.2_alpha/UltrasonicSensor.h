#include <sys/_stdint.h>
/**
 * @file UltrasonicSensor.h
 * @brief Header file for the UltrasonicSensor class, enabling distance measurement.
 *
 * The UltrasonicSensor class interfaces with an ultrasonic distance sensor, providing
 * functionality to initiate distance measurements, process echo signals, and retrieve
 * distance readings. It encapsulates the timing and signal processing required to use
 * ultrasonic sensors effectively in a variety of applications.
 *
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

class UltrasonicSensor
{
public:
    UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin, uint16_t max_distance);
    UltrasonicSensor(pin_size_t trigger_pin, pin_size_t echo_pin);
    ~UltrasonicSensor();

    void begin();
    void end();
    void update();
    void startMeasurement();
    bool isUpdating();
    bool detectedPeak(uint16_t threshold_distance, uint8_t consecutive_matches);
    uint16_t readDistance();

private:
    MovingAverage<uint16_t, uint16_t> filter;
    bool enabled;
    bool is_updating;
    pin_size_t trigger_pin;
    pin_size_t echo_pin;
    uint8_t state;
    uint16_t distance;
    uint16_t last_valid_distance;
    uint16_t max_distance;
};

#endif // ULTRASONICSENSOR_H