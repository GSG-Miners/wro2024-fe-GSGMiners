#include <sys/_stdint.h>
/**
 * @file Config.h
 * @brief Header file containing configuration enums for directional control and pin assignments.
 * 
 * This configuration file serves as a central repository for enumerating various constants
 * used throughout the application. It includes directional definitions for motor control,
 * pin assignments for interfacing with hardware components, and threshold values for
 * decision-making in navigation or race conditions. By centralizing these values, the code
 * becomes more maintainable and scalable, allowing for easy updates and modifications to
 * the system's configuration.
 *  
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef CONFIG_H
#define CONFIG_H

/**
 * @enum Pins
 * @brief Enumerates the microcontroller pin assignments for various components.
 * 
 * Provides a centralized and organized way of managing pin assignments throughout
 * the codebase. Each pin is associated with a specific function or component, such
 * as triggering ultrasonic sensors or controlling motor direction.
 */
enum Pins : const pin_size_t {
  TRIGGER_PIN_LEFT = 4,
  ECHO_PIN_LEFT = 3,
  TRIGGER_PIN_FRONT = 9,
  ECHO_PIN_FRONT = 8,
  TRIGGER_PIN_RIGHT = 17,
  ECHO_PIN_RIGHT = 16,
  VOLTAGE_MEASUREMENT_PIN = 14,
  BUTTON_PIN = 15,
  RELAY_PIN = 2,
  MOTOR_FORWARD_PIN = 5,
  MOTOR_BACKWARD_PIN = 6,
  SERVO_PIN = 7
};

/**
 * @enum Constants
 * @brief Enumerates constant parameters specific to racing or navigation thresholds.
 * 
 * Defines critical threshold values that are used in decision-making processes during
 * a race or navigation task. These values are used to determine when to turn, stop,
 * or adjust the vehicle's course based on sensor readings.
 */
enum Constants : const uint8_t {
  DEFAULT_SPEED = 60,  // SAFE 60
  REDUCED_SPEED = 55,
  MAX_LEFT = 46,
  MAX_RIGHT = 124,
  STRAIGHT = 90,
  MIN_DISTANCE = 15,
  // MAX_DISTANCE = 180, // OBSTACLE RACE
  MAX_DISTANCE = 130,
  STOPPING_OFFSET = 15  // SAFE QUALIFYING RACE
  // STOPPING_OFFSET = 10  // OBSTACLE RACE
};

/**
 * @enum Mode
 * @brief Enumerates the operational modes for the vehicle's navigation system.
 * 
 * Defines the operational states of the vehicle's navigation system,
 * particularly focusing on whether certain features are enabled or disabled. It is
 * used to toggle the inclusion of obstacle detection and the activation of the parking
 * assistance feature.
 */
enum Mode : const bool {
  OBSTACLES_INCLUDED = true,
  PARKING_ENABLED = false
};


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Symbolic Names
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @enum Direction
 * @brief Enumerates possible rotational directions for a motor or servo.
 * 
 * Defines symbolic names for the different rotational directions that can be used
 * to control motors or servos. This enumeration simplifies the code by replacing
 * numeric or boolean values with clear, descriptive identifiers.
 */
enum class Direction : const uint8_t {
  NONE,
  CLOCKWISE,
  ANTICLOCKWISE
};

/**
 * @enum TurnMode
 * @brief Enumerates the different modes of turning behavior for a vehicle.
 *
 * Defines the various strategies a vehicle can employ when navigating turns.
 * This enumeration allows for the flexible adaptation of turning maneuvers to
 * match the conditions of the track and the desired responsiveness of the vehicle.
 */
enum class TurnMode : const uint8_t {
  SHARP,
  SWIFT
};

enum class TurnDirection : const uint8_t {
  NONE,
  LEFT,
  RIGHT
};

#endif  // CONFIG_H