/**
 * @file demo_007_controller_v1.3c_updated_prototype.ino
 * @brief Updated both turning and section counting mechanism.
 * @author Maximilian Kautzsch
 * @date Created on 30th March 2024
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include "UltrasonicSensor.h"
#include "Button.h"
#include "Display.h"
#include "L298N.h"
#include "Controlling.h"

///----------------------------------------
/// @section CONFIG
///----------------------------------------


///----------------------------------------
/// @subsection ENUMS
///----------------------------------------

/**
 * @enum Direction
 * @enum Enum to store names for the directions.
*/
enum Direction : const uint8_t {
  NONE,
  CLOCKWISE,
  ANTICLOCKWISE
};

/**
 * @enum Pins
 * @brief Enum to hold the pin numbers for the various components.
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
  MOTOR_FORWARD_PIN = 5,
  MOTOR_BACKWARD_PIN = 6,
  SERVO_PIN = 7
};

/**
 * @enum Thresholds
 * @brief Enum to hold race specific constant parameters.
*/
enum Thresholds : const uint8_t {
  MAX_LEFT = 47,
  MAX_RIGHT = 124,
  STRAIGHT = 90,
  MIN_DISTANCE = 10,
  MAX_DISTANCE = 130
};


///----------------------------------------
/// @subsection STRUCTURES
///----------------------------------------

/**
 * @struct Parameters
 * @brief Struct to group all global parameter values.
 */
struct Parameters {
  uint8_t voltage;
  uint16_t distance_left;
  uint16_t distance_front;
  uint16_t distance_right;
  int16_t yaw_angle;
  int16_t angular_velocity;
  uint16_t steering_angle;
};

/**
 * @struct Race
 * @brief Struct to group all race-specified global variables.
 */
struct Race {
  Direction direction = NONE;
  uint8_t sections;
  uint8_t laps;
  int16_t setpoint_angle;
};

Parameters last;
Parameters current;
Parameters initial;
Race race;

///----------------------------------------
/// @subsection OBJECTS
///----------------------------------------

// Initialize the sensor and button objects
UltrasonicSensor sonarLeft(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT);
UltrasonicSensor sonarFront(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT);
UltrasonicSensor sonarRight(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT);
MPU6050 imu(Wire);
L298N motor(Pins::MOTOR_FORWARD_PIN, Pins::MOTOR_BACKWARD_PIN);
Servo servo;
Display display;

// Initialize the controller
DoubleSetpointController controller(ControllerDirection::DIRECT);

///----------------------------------------
/// @subsection SETUP
///----------------------------------------

/**
 * @brief Setup function for the Arduino sketch.
 */
void setup() {
  // Init communication protocols
  Serial.begin(9600);
  Wire.begin();

  // Init sensors
  imu.begin();
  sonarLeft.begin();
  sonarFront.begin();
  sonarRight.begin();

  // Init actuators
  motor.begin();
  servo.attach(Pins::SERVO_PIN);

  // Init controller
  controller.begin();
  controller.setStates(85, 90, 105);
  controller.setHysteresis(1);
}


///----------------------------------------
/// @section MAIN PROGRAM
///----------------------------------------

/**
 * @brief Main loop function for the Arduino sketch.
 */
void loop() {

  // Initialize the robot
  begin();

  // Update all of the sensors
  sonarsUpdate();
  imu.update();
  current.yaw_angle = imu.getAngleZ() - initial.yaw_angle;
  current.angular_velocity = imu.getGyroZ();
  current.voltage = map(analogRead(Pins::VOLTAGE_MEASUREMENT_PIN), 0, 1023, 0, 100);

  // Steering angle has changed
  if (current.steering_angle != last.steering_angle) {
    // Increment the amount of completed sections if robot just turned
    if ((last.steering_angle == Thresholds::MAX_LEFT || last.steering_angle == MAX_RIGHT)
        && current.steering_angle != Thresholds::MAX_LEFT
        && current.steering_angle != Thresholds::MAX_RIGHT) {
      race.sections++;
      if (race.direction == Direction::ANTICLOCKWISE) {
        race.setpoint_angle += 90;
      } else if (race.direction == Direction::CLOCKWISE) {
        race.setpoint_angle -= 90;
      }
    }
    // Transfer angle to the servo motor
    last.steering_angle = current.steering_angle;
    servo.write(current.steering_angle);
  }

  // Main algorithm
  if (getSections() == 0 && race.direction == NONE) {
    centreSteering();
    getDirection();
  } else if (getLaps() == 3) {
    stop();
  } else {
    // Differenciate between straight and curve sections
    if (detectedGap() && abs(current.yaw_angle) <= abs(race.setpoint_angle) - 20) {
      turn();
    } else {
      wallSteering();
    }
  }
}

///----------------------------------------
/// @section ALGORITHMS
///----------------------------------------


///----------------------------------------
/// @subsection RACE-SPECIFIC METHODS
///----------------------------------------

/**
 * @brief Checks whether the robot will collide soon.
 * @return True, if the robot collides.
*/
bool collisionRisk() {
  if (current.distance_front <= Thresholds::MIN_DISTANCE
      || current.distance_left <= Thresholds::MIN_DISTANCE
      || current.distance_right <= Thresholds::MAX_DISTANCE) {
    return true;
  } else {
    return false;
  }
}

/**
 @brief Gets the amount of sections completed by the robot.
 @return The amount of completed sections.
*/
uint8_t getSections() {
  return race.sections;
}

/**
 * @brief Gets the number of laps completed by the robot.
 * @return The amount of completed rounds.
*/
uint8_t getLaps() {
  race.laps = race.sections / 4;
  return race.laps;
}

/**
 * @brief Checks if there is a large gap to the left or the right.
 * @return True if large horizontal distance is detected.
*/
bool detectedGap() {
  if (race.direction == Direction::ANTICLOCKWISE) {
    if (current.distance_left >= Thresholds::MAX_DISTANCE) {
      return true;
    } else {
      return false;
    }
  } else if (race.direction == Direction::CLOCKWISE) {
    if (current.distance_right >= Thresholds::MAX_DISTANCE) {
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief Gets the race.direction based on whether the gap is to the left or the right side.
 * @return The race.direction.
*/
void getDirection() {
  static bool direction_is_determined;

  if (!direction_is_determined) {
    if (current.distance_left >= Thresholds::MAX_DISTANCE) {
      race.direction = Direction::ANTICLOCKWISE;
      race.setpoint_angle = 90;
      current.steering_angle = Thresholds::MAX_LEFT;
      direction_is_determined = true;
    } else if (current.distance_right >= Thresholds::MAX_DISTANCE) {
      race.direction = Direction::CLOCKWISE;
      race.setpoint_angle = -90;
      current.steering_angle = Thresholds::MAX_RIGHT;
      direction_is_determined = true;
    }
  }
}

///----------------------------------------
/// @subsection MANOEUVRES
///----------------------------------------

/**
 * @brief Steers towards the corresponding wall, depending
 * on the race.direction.
*/
void wallSteering() {
  // Transfer the default speed to the motor
  motor.write(100);

  // Calculate and store the output of the control loop
  if (race.direction == Direction::ANTICLOCKWISE) {
    controller.setDirection(ControllerDirection::DIRECT);
    controller.setSetpoint(40);
    current.steering_angle = controller.getOutput(current.distance_left);
  } else if (race.direction == Direction::CLOCKWISE) {
    controller.setDirection(ControllerDirection::REVERSE);
    controller.setSetpoint(45);
    controller.getOutput(current.distance_right);
  }
}

/**
 * @brief Tries to avoid a collision with a wall or obstacle.
 * 
 * Sets the steering angle to the maximum and lowers the motor
 * speed.
*/
void avoidCollision() {
  if (current.distance_front <= Thresholds::MIN_DISTANCE) {
    motor.write(-60);
  }
  if (current.distance_left <= Thresholds::MIN_DISTANCE) {
    motor.write(60);
    current.steering_angle = Thresholds::MAX_RIGHT;
  }
  if (current.distance_right <= Thresholds::MIN_DISTANCE) {
    motor.write(60);
    current.steering_angle = Thresholds::MAX_LEFT;
  }
}

/**
 * @brief Captures intial measurement values.
*/
void begin() {
  static bool is_initialized;

  if (!is_initialized) {
    imu.calcGyroOffsets();
    imu.update();
    initial.yaw_angle = imu.getAngleZ();
    initial.distance_front = sonarFront.readDistance();
    is_initialized = true;
  }
}

/**
 * @brief Shuts the entire robot down.
 */
void stop() {
  current.steering_angle = Thresholds::STRAIGHT;
  motor.end();
}

/**
 * @brief Manoeuvre for turning left or right, depending on the race.direction.
*/
void turn() {
  // Transfer a lowered speed to the motor
  motor.write(70);

  // Set the steering angle to the maximum of the corresponding race.direction
  if (race.direction == Direction::ANTICLOCKWISE) {
    current.steering_angle = Thresholds::MAX_LEFT;
  } else if (race.direction == Direction::CLOCKWISE) {
    current.steering_angle = Thresholds::MAX_RIGHT;
  }
}


///----------------------------------------
/// @subsection CONTROL LOOPS
///----------------------------------------

/**
 * @brief Robot drives in the middle of the straight section.
*/
void centreSteering() {
  // Transfer the default speed to the motor
  motor.write(100);

  // Calculate and store the output of the control loop
  controller.setDirection(ControllerDirection::DIRECT);
  controller.setSetpoint(current.distance_right);
  current.steering_angle = controller.getOutput(current.distance_left);
}

/**
 * @brief Tries to keep the angular velocity around the z-axis at 0.
*/
void gyroSteering() {
  // Transfer the default speed to the motor
  motor.write(100);

  // Calculate and store the output of the control loop
  controller.setDirection(ControllerDirection::REVERSE);
  controller.setSetpoint(0);
  current.steering_angle = controller.getOutput(current.angular_velocity);
}

///----------------------------------------
/// @section MODULE ENHANCEMENTS
///----------------------------------------

/**
 * @brief Measures the distance to the front, left and right.
 * Measurements are performed individually every 10 ms.
*/
void sonarsUpdate() {
  static unsigned long last_millis;
  static uint8_t state;

  switch (state) {
    case 0:
      {
        current.distance_left = sonarLeft.readDistance();
      }
      break;
    case 1:
      {
        current.distance_front = sonarFront.readDistance();
      }
      break;
    case 2:
      {
        current.distance_right = sonarRight.readDistance();
      }
      break;
    default:
      {
        state = 0;
      }
      break;
  }

  if (millis() - last_millis > 10) {
    last_millis = millis();
    state++;
  }
}

/**
 * @brief Function to print the sensor readings on the LCD display.
 */
void showData() {
  static unsigned long last_millis;

  if (millis() - last_millis > 200) {
    last_millis = millis();

    // Print the display preset
    display.preset();

    // Update the data on the display
    display.update(last.distance_left, current.distance_left, 1, 0, 2, false);
    display.update(last.distance_front, current.distance_front, 5, 0, 2, false);
    display.update(last.distance_right, current.distance_right, 9, 0, 2, false);
    display.update(last.yaw_angle, current.yaw_angle, 12, 0, 4, false);
    display.update(last.voltage, current.voltage, 14, 1, 2, false);
  }
}