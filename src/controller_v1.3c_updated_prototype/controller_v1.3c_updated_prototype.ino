/**
 * @file demo_007_controller_v1.3c_updated_prototype.ino
 * @brief Updated both turning and section counting mechanism.
 * @author Maximilian Kautzsch
 * @date Created on 30th March 2024
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <NewPing.h>
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
enum Direction : const uint8_t
{
  NONE,
  CLOCKWISE,
  ANTICLOCKWISE
};

/**
 * @enum Pins
 * @brief Enum to hold the pin numbers for the various components.
 */
enum Pins : const pin_size_t
{
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
enum Thresholds : const uint8_t
{
  MAX_LEFT = 47,
  MAX_RIGHT = 124,
  STRAIGHT = 90,
  MIN_DISTANCE = 10,
  MAX_DISTANCE = 200
};

///----------------------------------------
/// @subsection STRUCTURES
///----------------------------------------

/**
 * @struct Parameters
 * @brief Struct to group all global parameter values.
 */
struct Parameters
{
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
struct Race
{
  Direction direction;
  bool just_turned;
  uint8_t sections;
  uint8_t laps;
  int16_t setpoint_angle;
};

static Parameters last;
Parameters current;
static Parameters initial;
static Race race;

///----------------------------------------
/// @subsection OBJECTS
///----------------------------------------

#define SONAR_NUM 3
#define LEFT 0
#define FRONT 1
#define RIGHT 2

// Initialize the sensor and button objects
NewPing sonar[SONAR_NUM] = {
    NewPing(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT, 400),   // Left sonar
    NewPing(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT, 400), // Front sonar
    NewPing(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT, 400)  // Right sonar
};
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
void setup()
{
  // Init communication protocols
  Serial.begin(19200);
  Wire.begin();

  // Init sensors
  imu.begin();

  // Init actuators
  motor.begin();
  servo.attach(Pins::SERVO_PIN);

  // Init controller
  controller.begin();
  controller.setStates(85, 90, 105);
  controller.setHysteresis(1);

  // Init direction
  race.direction = NONE;
}

///----------------------------------------
/// @section MAIN PROGRAM
///----------------------------------------

/**
 * @brief Main loop function for the Arduino sketch.
 */
void loop()
{
  // Initialize the robot
  begin();

  // Update the measurement values
  updateSensors();

  // Update the actuator parameters
  updateActuators();

  Serial.print("Left:");
  Serial.print(current.distance_left);
  Serial.print("\t");
  Serial.print("Front:");
  Serial.print(current.distance_front);
  Serial.print("\t");
  Serial.print("Right:");
  Serial.print(current.distance_right);
  Serial.print("\n");

  // Main algorithm
  if (abs(current.yaw_angle) <= 90 && race.direction == NONE)
  {
    centreSteering();
    getDirection();
  }
  else if (abs(current.yaw_angle) >= 970 && abs(current.yaw_angle) <= 1060)
  {
    centreSteering();
  }
  else if (abs(current.yaw_angle) >= 1060 && current.distance_front >= initial.distance_front - 10 && current.distance_front <= initial.distance_front + 10)
  {
    stop();
  }
  else
  {
    // Differenciate between straight and curve sections
    if (detectedGap())
    {
      turn();
    }
    else
    {
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
bool collisionRisk()
{
  if (current.distance_front <= Thresholds::MIN_DISTANCE || current.distance_left <= Thresholds::MIN_DISTANCE || current.distance_right <= Thresholds::MAX_DISTANCE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 @brief Gets the amount of sections completed by the robot.
 @return The amount of completed sections.
*/
uint8_t getSections()
{
  return race.sections;
}

/**
 * @brief Gets the number of laps completed by the robot.
 * @return The amount of completed rounds.
 */
uint8_t getLaps()
{
  race.laps = race.sections / 4;
  return race.laps;
}

/**
 * @brief Checks if there is a large gap to the left or the right.
 * @return True if large horizontal distance is detected.
 */
bool detectedGap()
{
  if (race.direction == Direction::ANTICLOCKWISE)
  {
    if (current.distance_left >= Thresholds::MAX_DISTANCE)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (race.direction == Direction::CLOCKWISE)
  {
    if (current.distance_right >= Thresholds::MAX_DISTANCE)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

/**
 * @brief Gets the race.direction based on whether the gap is to the left or the right side.
 * @return The race.direction.
 */
void getDirection()
{
  if (current.distance_left >= MAX_DISTANCE)
  {
    race.direction = Direction::ANTICLOCKWISE;
    race.setpoint_angle = 90;
    current.steering_angle = Thresholds::MAX_LEFT;
  }
  else if (current.distance_right >= MAX_DISTANCE)
  {
    race.direction = Direction::CLOCKWISE;
    race.setpoint_angle = -90;
    current.steering_angle = Thresholds::MAX_RIGHT;
  }
}

///----------------------------------------
/// @subsection MANOEUVRES
///----------------------------------------

/**
 * @brief Steers towards the corresponding wall, depending
 * on the race.direction.
 */
void wallSteering()
{
  // Transfer the default speed to the motor
  motor.write(100);

  // Calculate and store the output of the control loop
  if (race.direction == Direction::ANTICLOCKWISE)
  {
    controller.setDirection(ControllerDirection::DIRECT);
    controller.setSetpoint(40);
    current.steering_angle = controller.getOutput(current.distance_left);
  }
  else if (race.direction == Direction::CLOCKWISE)
  {
    controller.setDirection(ControllerDirection::REVERSE);
    controller.setSetpoint(45);
    current.steering_angle = controller.getOutput(current.distance_right);
  }
}

/**
 * @brief Tries to avoid a collision with a wall or obstacle.
 *
 * Sets the steering angle to the maximum and lowers the motor
 * speed.
 */
void avoidCollision()
{
  if (current.distance_front <= Thresholds::MIN_DISTANCE)
  {
    motor.write(-60);
  }
  if (current.distance_left <= Thresholds::MIN_DISTANCE)
  {
    motor.write(60);
    current.steering_angle = Thresholds::MAX_RIGHT;
  }
  if (current.distance_right <= Thresholds::MIN_DISTANCE)
  {
    motor.write(60);
    current.steering_angle = Thresholds::MAX_LEFT;
  }
}

/**
 * @brief Captures intial measurement values.
 */
void begin()
{
  static bool is_initialized;

  if (!is_initialized)
  {
    imu.calcGyroOffsets();
    imu.update();
    initial.yaw_angle = imu.getAngleZ();
    initial.distance_front = sonar[FRONT].ping_cm();
    is_initialized = true;
  }
}

/**
 * @brief Shuts the entire robot down.
 */
void stop()
{
  current.steering_angle = Thresholds::STRAIGHT;
  motor.end();
  servo.detach();
}

/**
 * @brief Manoeuvre for turning left or right, depending on the race.direction.
 */
void turn()
{
  // Transfer a lowered speed to the motor
  motor.write(70);

  // Set the steering angle to the maximum of the corresponding race.direction
  if (race.direction == Direction::ANTICLOCKWISE)
  {
    current.steering_angle = Thresholds::MAX_LEFT;
  }
  else if (race.direction == Direction::CLOCKWISE)
  {
    current.steering_angle = Thresholds::MAX_RIGHT;
  }
}

///----------------------------------------
/// @subsection CONTROL LOOPS
///----------------------------------------

/**
 * @brief Robot drives in the middle of the straight section.
 */
void centreSteering()
{
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
void gyroSteering()
{
  // Transfer the default speed to the motor
  motor.write(100);

  // Calculate and store the output of the control loop
  controller.setDirection(ControllerDirection::REVERSE);
  controller.setSetpoint(race.setpoint_angle);
  current.steering_angle = controller.getOutput(current.yaw_angle);
}

///----------------------------------------
/// @section MODULE ENHANCEMENTS
///----------------------------------------

/**
 * @brief Updates all of the measurement values.
 *
 * Sonars, gyroscope and voltage pin are refreshed.
 */
void updateSensors()
{
  updateSonars();
  imu.update();
  current.yaw_angle = imu.getAngleZ() - initial.yaw_angle;
  current.angular_velocity = imu.getGyroZ();
  current.voltage = map(analogRead(Pins::VOLTAGE_MEASUREMENT_PIN), 0, 1023, 0, 100);
}

/**
 * @brief Updates all of the actuator parameters.
 *
 * Refreshes the steering angle value if it changed.
 * Moreover, certain race-specific parameters are refreshed.
 */
void updateActuators()
{
  // Steering angle has changed
  if (current.steering_angle != last.steering_angle)
  {
    // Increment the amount of completed sections if robot just turned
    if (last.steering_angle == Thresholds::MAX_LEFT && current.steering_angle != Thresholds::MAX_LEFT)
    {
      race.sections++;
      race.setpoint_angle += 90;
      race.just_turned = true;
    }
    else if (last.steering_angle == Thresholds::MAX_RIGHT && current.steering_angle != Thresholds::MAX_RIGHT)
    {
      race.sections++;
      race.setpoint_angle -= 90;
      race.just_turned = true;
    }
    else
    {
      race.just_turned = false;
    }

    // Transfer angle to the servo motor
    last.steering_angle = current.steering_angle;
    servo.write(current.steering_angle);
  }
}

/**
 * @brief Filters out peaks that occur consecutively over a specified threshold.
 *
 * @param input The current input value to check.
 * @param threshold The value above which a peak is considered.
 * @param consecutive_matches The number of consecutive inputs needed to consider as a peak.
 * @return true if a peak is detected, false otherwise.
 */
bool peak(uint16_t input, uint16_t threshold, uint8_t consecutive_matches)
{
  static uint8_t matches;

  if (input >= threshold)
  {
    if (matches < consecutive_matches)
    {
      matches++;
    }
    if (matches >= consecutive_matches)
    {
      return true;
    }
  }
  else
  {
    matches = 0;
    return false;
  }
}

/**
 * @brief Measures the distance to the front, left and right.
 *
 * Measurement for each is performed individually every 10 ms.
 */
void updateSonars()
{
  uint16_t frequency = 33;
  static unsigned long last_millis;
  static uint8_t state;

  switch (state)
  {
  case LEFT:
  {
    current.distance_left = sonar[LEFT].ping_cm();
    if (millis() - last_millis > 1000 / (3 * frequency))
    {
      last_millis = millis();
      state++;
    }
  }
  break;
  case FRONT:
  {
    current.distance_front = sonar[FRONT].ping_cm();
    if (millis() - last_millis > 1000 / (3 * frequency))
    {
      last_millis = millis();
      state++;
    }
  }
  break;
  case RIGHT:
  {
    current.distance_right = sonar[RIGHT].ping_cm();
    if (millis() - last_millis > 1000 / (3 * frequency))
    {
      last_millis = millis();
      state++;
    }
  }
  break;
  default:
  {
    state = LEFT;
  }
  break;
  }
}

/**
 * @brief Function to print the sensor readings on the LCD display.
 */
void showData()
{
  static unsigned long last_millis;

  if (millis() - last_millis > 200)
  {
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