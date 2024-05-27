/**
 * @file UltrasonicSensor.cpp
 * @brief Implementation of the UltrasonicSensor class.
 */

#include "UltrasonicSensor.h"
#include <sys/_stdint.h>

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
void UltrasonicSensor::begin()
{
    pinMode(this->trigger_pin, OUTPUT);
    pinMode(this->echo_pin, INPUT);
    this->filter.begin();
    this->enabled = true;
    this->state = 0;
    this->is_updating = 0;
}

/**
 * @brief Disables the sensor.
 *
 * Ends the filter.
 */
void UltrasonicSensor::end()
{
    this->filter.end();
    this->enabled = false;
}

/**
 * @brief Updates the sensor measurements at room temperature.
 *
 * This method progresses through a state machine to update the distance measurement
 * from the ultrasonic sensor. It sends out an ultrasonic pulse and listens for its echo
 * to calculate the distance to an object.
 */
void UltrasonicSensor::update()
{
    if (!this->enabled)
        return;

    static unsigned long last_micros;

    // Guard clause to exit if no update is in progress.
    if (!this->is_updating)
        return;

    // State machine for updating the sensor reading.
    switch (this->state)
    {
    case 0: // State 0: Trigger the ultrasonic pulse.
    {
        last_micros = micros();
        digitalWrite(trigger_pin, HIGH);
        this->state++;
    }
    break;
    case 1: // State 1: End the ultrasonic pulse after 15 microseconds.
    {
        if (micros() > last_micros + 15)
        {
            digitalWrite(trigger_pin, LOW);
            this->state++;
        }
    }
    break;
    case 2: // State 2: Wait for the echo or timeout.
    {
        if (micros() > last_micros + 29310)
        {
            this->distance = max_distance;
            this->state = 0;
            this->is_updating = false;
        }
        else
        {
            if (digitalRead(echo_pin) == HIGH)
            {
                last_micros = micros();
                this->state++;
            }
        }
    }
    break;
    case 3: // State 3: Calculate the distance based on the echo pulse width.
    {
        unsigned long pulse_width = micros() - last_micros;
        if (digitalRead(echo_pin) == LOW || pulse_width > 25000)
        {
            pulse_width /= 2;
            if (pulse_width < 12500)
            {
                this->distance = pulse_width / 29.1;
            }
            this->state = 0;
            this->is_updating = false;
        }
    }
    break;
    }
}

/**
 * @brief Initiates a new measurement cycle for the ultrasonic sensor.
 *
 * This function prepares the ultrasonic sensor to start a new measurement by setting
 * the 'is_updating' flag to true. This flag is used to indicate that a measurement
 * process is currently underway and that the sensor's state should be considered
 * volatile until the measurement is complete.
 *
 * Additionally, the function resets the 'state' variable to 0. The 'state' variable
 * is typically used to track the progress through different stages of the measurement
 * process. By setting it to 0, the function ensures that the sensor starts from the
 * initial stage of the measurement cycle.
 */
void UltrasonicSensor::startMeasurement()
{
    if (!this->enabled)
        return;

    this->is_updating = true; // Flag the start of a measurement update.
    this->state = 0;          // Reset the state to the initial value.
}

/**
 * @brief Indicates whether the sensor is currently updating its measurement.
 *
 * @return True if the sensor is in the process of updating, false otherwise.
 */
bool UltrasonicSensor::isUpdating()
{
    if (!this->enabled)
        return 0;

    return this->is_updating;
}

/**
 * @brief Checks if a peak is detected by the sensor.
 *
 * Compares the current distance measurement to a threshold value to determine if a peak
 * (an object within a certain range) is detected. Requires a specified number of consecutive
 * matches to confirm the detection and avoid false positives.
 *
 * @param threshold_distance The threshold distance for peak detection.
 * @param consecutive_matches The number of consecutive matches required for peak detection.
 * @return True if a peak is detected, false otherwise.
 */
bool UltrasonicSensor::detectedPeak(uint16_t threshold_distance, uint8_t consecutive_matches)
{
    if (!this->enabled)
        return 0;

    return filter.detectedPeak(threshold_distance, consecutive_matches);
}

/**
 * @brief Reads the distance from the sensor at room temperature.
 *
 * Returns the most recent distance measurement obtained by the sensor. Assumes that the
 * measurement is taken at room temperature, which affects the speed of sound and thus the
 * distance calculation.
 *
 * @return The distance in centimeters.
 */
uint16_t UltrasonicSensor::readDistance()
{
    if (!this->enabled)
        return 0;

    return this->distance;
}