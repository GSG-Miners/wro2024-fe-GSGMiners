/**
 * @file demo_008_controller_v3.ino
 * @brief This sketch is used to test out an initial three point controller.
<<<<<<< HEAD
 * @date 23rd December 2023 - 31st January 2024
=======
 * @date 23rd December 2023 - 3rd February 2024
>>>>>>> 7156fcf (Large Update)
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include "ultrasonic_sensor.h"
#include "button.h"
#include "lcd_display.h"
#include "dc_motor.h"
#include "controlling.h"

<<<<<<< HEAD
///----------------------------------------
/// @section CONFIG
///----------------------------------------
=======

///========================================
/// @section CONFIG
///========================================

>>>>>>> 7156fcf (Large Update)

///----------------------------------------
/// @subsection ENUMS
///----------------------------------------

/**
 * @enum Pins
 * @brief Enum to hold the pin numbers for the various components.
 */
<<<<<<< HEAD
enum Pins : const pin_size_t
{
=======
enum Pins : const pin_size_t {
>>>>>>> 7156fcf (Large Update)
  TRIGGER_PIN_LEFT = 4,
  ECHO_PIN_LEFT = 3,
  TRIGGER_PIN_FRONT = 9,
  ECHO_PIN_FRONT = 8,
  TRIGGER_PIN_RIGHT = 17,
  ECHO_PIN_RIGHT = 16,
  VOLTMETER_PIN = 14,
  BUTTON_PIN = 15,
  MOTOR_FORWARD_PIN = 5,
  MOTOR_BACKWARD_PIN = 6,
<<<<<<< HEAD
  SERVO_PIN = 7,
};

/**
 * @enum Direction
 * @enum Enum to store names for the directions.
 */
enum Direction : const bool
{
=======
  SERVO_PIN = 7
};

/**
  * @enum Direction
  * @enum Enum to store names for the directions.
*/
enum Direction : const bool {
>>>>>>> 7156fcf (Large Update)
  CLOCKWISE = false,
  ANTI_CLOCKWISE = true
};

/**
<<<<<<< HEAD
 * @enum SteeringAngles
 * @brief Enum to hold the max values of the steering angles.
 */
enum SteeringAngles : const uint8_t
{
=======
  * @enum SteeringAngles
  * @brief Enum to hold the max values of the steering angles.
*/
enum SteeringAngles : const uint8_t {
>>>>>>> 7156fcf (Large Update)
  MAX_LEFT = 47,
  MAX_RIGHT = 124,
  STRAIGHT = 90
};

/**
<<<<<<< HEAD
 * @enum RaceConstants
 * @brief Enum to hold race specific constant parameters.
 */
enum RaceConstants : const uint8_t
{
=======
  * @enum RaceDistances
  * @brief Enum to hold race specific distance values.
*/
enum RaceDistances : const uint8_t {
  SETPOINT_DISTANCE = 40,
>>>>>>> 7156fcf (Large Update)
  MIN_DISTANCE = 5,
  MAX_DISTANCE = 130
};

<<<<<<< HEAD
=======

>>>>>>> 7156fcf (Large Update)
///----------------------------------------
/// @subsection STRUCTURES
///----------------------------------------

/**
<<<<<<< HEAD
 * @struct SensorReadings
 * @brief Struct to group all input values (from the sensors).
 */
typedef struct
{
  uint8_t button_count;     ///< Current button count
  uint8_t voltage;          ///< Current voltage reading
  uint16_t distance_left;   ///< Current distance reading from the left sensor
  uint16_t distance_front;  ///< Current distance reading from the front sensor
  uint16_t distance_right;  ///< Current distance reading from the right sensor
  int16_t yaw_angle;        ///< Current yaw angle reading
  int16_t angular_velocity; ///< Current raw gyroscope data (angular velocity)
=======
  * @struct SensorReadings
  * @brief Struct to group all input values (from the sensors).
*/
typedef struct {
  uint8_t button_count;      ///< Current button count
  uint8_t voltage;           ///< Current voltage reading
  uint16_t distance_left;    ///< Current distance reading from the left sensor
  uint16_t distance_front;   ///< Current distance reading from the front sensor
  uint16_t distance_right;   ///< Current distance reading from the right sensor
  int16_t yaw_angle;         ///< Current yaw angle reading
  int16_t angular_velocity;  ///< Current raw gyroscope data (angular velocity)
>>>>>>> 7156fcf (Large Update)
} SensorReadings;

/**
  * @struct ControllerParameters
  * @brief Struct to store all attributes that are used to configure a three state controller
 or PID controller.
*/
<<<<<<< HEAD
typedef struct
{
  float proportional_gain;       ///< Gain of proportional term
  float integral_gain;           ///< Gain of integral term
  float derivative_gain;         ///< Gain of derivative term
  ControllerDirection direction; ///< Direction of the controller (direct or reverse)
} ControllerParameters;

=======
typedef struct {
  float proportional_gain;        ///< Gain of proportional term
  float integral_gain;            ///< Gain of integral term
  float derivative_gain;          ///< Gain of derivative term
  ControllerDirection direction;  ///< Direction of the controller (direct or reverse)
} ControllerParameters;

///========================================
/// @section OBJECT INITIALIZATION
///========================================

>>>>>>> 7156fcf (Large Update)
// Initialize enum and struct instances
SensorReadings lastReadings;
SensorReadings currentReadings;
SensorReadings initialReadings;
<<<<<<< HEAD
Direction direction;

// Controlling configurations
ControllerParameters paramsAnticlockwiseSteering = {0.00, 0.00, 0.00, ControllerDirection::kDirect};
ControllerParameters paramsClockwiseSteering = {0.00, 0.00, 0.00, ControllerDirection::kReverse};
ControllerParameters paramsNeutralSteering = {0.00, 0.00, 0.00, ControllerDirection::kDirect};
ControllerParameters paramsGyroSteering = {0.00, 0.00, 0.00, ControllerDirection::kDirect};
ControllerParameters paramsDriveStraightPID = {3.00, 1.20, 0.50, ControllerDirection::kDirect};

///----------------------------------------
/// @subsection OBJECTS
///----------------------------------------

// Initialize the sensor and button objects
Button button(Pins::BUTTON_PIN);
UltrasonicSensor sonarLeft(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT, SonarMode::kManual);
UltrasonicSensor sonarFront(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT, SonarMode::kManual);
UltrasonicSensor sonarRight(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT, SonarMode::kManual);
MPU6050 imu(Wire);
Motor motor(Pins::MOTOR_FORWARD_PIN, Pins::MOTOR_BACKWARD_PIN);
Servo servo;
PIDController driveStraightPID(paramsDriveStraightPID.direction);
ThreeStateController anticlockwiseSteering(paramsAnticlockwiseSteering.direction);
ThreeStateController clockwiseSteering(paramsClockwiseSteering.direction);
ThreeStateController neutralSteering(paramsNeutralSteering.direction);
ThreeStateController gyroscopeSteering(paramsGyroSteering.direction);

///----------------------------------------
/// @subsection SETUP
///----------------------------------------
=======
static Direction direction;

// Controlling configurations
ControllerParameters paramsNeutralSteering = { 0.00, 0.00, 0.00, ControllerDirection::DIRECT };
ControllerParameters paramsFinalSectionSteering = { 0.00, 0.00, 0.00, ControllerDirection::DIRECT };
ControllerParameters paramsAnticlockwiseSteering = { 0.00, 0.00, 0.00, ControllerDirection::DIRECT };
ControllerParameters paramsClockwiseSteering = { 0.00, 0.00, 0.00, ControllerDirection::REVERSE };
ControllerParameters paramsGyroSteering = { 0.00, 0.00, 0.00, ControllerDirection::DIRECT };

ControllerParameters paramsDriveStraightPID = { 3.00, 1.20, 0.50, ControllerDirection::DIRECT };

// Initialize the hardware objects
Button button(Pins::BUTTON_PIN);
UltrasonicSensor sonarLeft(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT, SonarMode::MANUAL);
UltrasonicSensor sonarFront(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT, SonarMode::MANUAL);
UltrasonicSensor sonarRight(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT, SonarMode::MANUAL);
MPU6050 imu(Wire);
Motor motor(Pins::MOTOR_FORWARD_PIN, Pins::MOTOR_BACKWARD_PIN);
Servo servo;

// Initialize the controller objects
PIDController driveStraightPID(paramsDriveStraightPID.direction);
ThreeStateController neutralSteering(paramsNeutralSteering.direction);
ThreeStateController anticlockwiseSteering(paramsAnticlockwiseSteering.direction);
ThreeStateController clockwiseSteering(paramsClockwiseSteering.direction);
ThreeStateController gyroscopeSteering(paramsGyroSteering.direction);


///========================================
/// @section SETUP FUNCTION
///========================================
>>>>>>> 7156fcf (Large Update)

/**
 * @brief Setup function for the Arduino sketch.
 */
<<<<<<< HEAD
void setup()
{
  Serial.begin(9600);
  Wire.begin();

=======
void setup() {
  // Serial and Wire Initialization
  Serial.begin(9600);
  Wire.begin();

  // Hardware Initialization
>>>>>>> 7156fcf (Large Update)
  imu.begin();
  lcd.init();
  lcd.backlight();

  servo.attach(Pins::SERVO_PIN);
  motor.init();
  motor.setAcceleration(100);

<<<<<<< HEAD
=======
  // Controller Initialization
>>>>>>> 7156fcf (Large Update)
  driveStraightPID.setLimits(SteeringAngles::MAX_LEFT, SteeringAngles::MAX_RIGHT);
  driveStraightPID.tune(paramsDriveStraightPID.proportional_gain, paramsDriveStraightPID.integral_gain, paramsDriveStraightPID.derivative_gain);
  driveStraightPID.setpoint(90);

<<<<<<< HEAD
  anticlockwiseSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  anticlockwiseSteering.setpoint(40);
  anticlockwiseSteering.setHysteresis(1);

  clockwiseSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  clockwiseSteering.setpoint(40);
  clockwiseSteering.setHysteresis(1);

  neutralSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  neutralSteering.setHysteresis(1);

=======
  neutralSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  neutralSteering.setHysteresis(1);

  anticlockwiseSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  anticlockwiseSteering.setpoint(RaceDistances::SETPOINT_DISTANCE);
  anticlockwiseSteering.setHysteresis(1);

  clockwiseSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  clockwiseSteering.setpoint(RaceDistances::SETPOINT_DISTANCE);
  clockwiseSteering.setHysteresis(1);

>>>>>>> 7156fcf (Large Update)
  gyroscopeSteering.setSteeringAngles(SteeringAngles::STRAIGHT, 85, 100);
  gyroscopeSteering.setpoint(0);
  gyroscopeSteering.setHysteresis(1);

<<<<<<< HEAD
=======
  // Initial LCD image
>>>>>>> 7156fcf (Large Update)
  lcd.setCursor(2, 0);
  lcd.print("PRESS BUTTON");
  lcd.setCursor(4, 1);
  lcd.print("TO START");
}

<<<<<<< HEAD
///----------------------------------------
/// @section MAIN PROGRAM
///----------------------------------------
=======

///========================================
/// @section MAIN PROGRAM
///========================================
>>>>>>> 7156fcf (Large Update)

/**
 * @brief Main loop function for the Arduino sketch.
 */
<<<<<<< HEAD
void loop()
{
=======
void loop() {
>>>>>>> 7156fcf (Large Update)

  // Update all of the measured values
  button.update();
  imu.update();
  sonarMeasurementSequence();
  motor.update();

<<<<<<< HEAD
  // Assign the modified sensor readings to variables
=======
  updateReadings();


  switch (currentReadings.button_count) {
    case 1:
      {
        static bool is_booted_up, direction_set;

        switch (is_booted_up) {
          // Bootup of the robot
          case false:
            {
              bootupRobot();
              is_booted_up = true;
            }
            break;

          // Main code for controlling the robot
          case true:
            {
              // Display the measured values on the LCD
              lcdPrintValues();

              // Get the direction initially
              if (!direction_set) {
                motor.setSpeed(80);
                neutralControlLoop();
                if (getDirection()) {
                  direction_set = true;
                }
              } else {
                // Automatic steering of the robot
                if (isFinalSection()) {
                  stop();
                } else {
                  autoSteering();
                }
              }
            }
            break;
        }
      }
      break;
    case 2:
      {
        stop();
      }
      break;
  }
}

///========================================
/// @section ALGORITHMS
///========================================


///----------------------------------------
/// @subsection UTILITY FUNCTIONS
///----------------------------------------

///----------------------------------------
/// @subsubsection GENERAL LOGIC
///----------------------------------------

/**
 * @brief Checks whether a given number_A lies within the range of the deviation of number_B.
 * @param number_A The number to be checked.
 * @param number_B The reference number for the range.
 * @param deviation The amount of deviation allowed from the reference number.
 * @return True if number_A is within the range, false otherwise.
*/
void isInRange(int16_t number_A, uint16_t number_B, uint16_t deviation) {
  int16_t lower_bound = number_A - deviation;
  int16_t upper_bound = number_B + deviation;

  return (number_A >= lower_bound && number_A <= upper_bound);
}


///----------------------------------------
/// @subsubsection COMMAND SEQUENCES
///----------------------------------------

/**
 * @brief Initizializes all of the sensor readings and boots up the LCD.
*/
void bootupRobot() {
  imu.calcOffsets();

  // Store the initial sensor readings in seperate structure
  initialReadings.yaw_angle = imu.getAngleZ();
  initialReadings.distance_front = sonarFront.getDistance();
  initialReadings.distance_left = sonarLeft.getDistance();
  initialReadings.distance_right = sonarRight.getDistance();

  // Bootup animation of the LCD
  lcdBootup();
  lcdClear();
}

/**
 * @brief Measures the distance to the front, left and right.
 * Measurements are performed individually every 10 ms.
*/
void sonarMeasurementSequence() {
  static unsigned long last_ms;
  static uint8_t measurement_phase;

  if (millis() - last_ms > 10) {
    last_ms = millis();

    switch (measurement_phase) {
      case 0:
        {
          sonarLeft.startMeasurement();
          sonarLeft.update();
          measurement_phase++;
        }
        break;
      case 1:
        {
          sonarFront.startMeasurement();
          sonarFront.update();
          measurement_phase++;
        }
        break;
      case 2:
        {
          sonarRight.startMeasurement();
          sonarRight.update();
          measurement_phase++;
        }
        break;
      default:
        {
          measurement_phase = 0;
        }
    }
  }
}

/**
 * @brief Method used for updating all of the current readings from the sensors.
*/
void updateReadings() {
>>>>>>> 7156fcf (Large Update)
  currentReadings.button_count = button.getCount();
  currentReadings.voltage = map(analogRead(Pins::VOLTMETER_PIN), 0, 1023, 0, 100);
  currentReadings.yaw_angle = imu.getAngleZ() - initialReadings.yaw_angle;
  currentReadings.angular_velocity = imu.getGyroZ();
  currentReadings.distance_front = sonarFront.getDistance();
  currentReadings.distance_left = sonarLeft.getDistance();
  currentReadings.distance_right = sonarRight.getDistance();
<<<<<<< HEAD

  if (currentReadings.button_count == 1)
  {
    static bool bootup_phase, direction_determined;

    switch (bootup_phase)
    {
    // Bootup of the robot
    case 0:
    {
      imu.calcOffsets();
      initialReadings.yaw_angle = imu.getAngleZ();

      lcdBootup();
      lcdClear();
      bootup_phase = !bootup_phase;
    }
    break;

    // Main code for controlling the robot
    case 1:
    {
      // Display the measured values on the LCD
      lcdPrintValues();

      // Get the direction initially
      if (!direction_determined)
      {
        getDirection();
        if (direction == Direction::ANTI_CLOCKWISE || direction == Direction::CLOCKWISE)
        {
          direction_determined = !direction_determined;
        }
        neutralControlLoop();
      }
      else
      {
        // Automatic steering of the robot
        autoSteering();
      }
    }
    break;
    }
  }
  else if (currentReadings.button_count == 2)
  {
    lcdShutdown();
    lcd.noBacklight();
    servo.write(SteeringAngles::STRAIGHT);
    motor.stop();
  }
}

///----------------------------------------
/// @section ALGORITHMS
///----------------------------------------

///----------------------------------------
/// @subsection HELPER FUNCTIONS
///----------------------------------------

/**
 * @brief Measures the distance to the front, left and right.
 * Measurements are performed individually every 10 ms.
 */
void sonarMeasurementSequence()
{
  static unsigned long last_ms;
  static uint8_t measurement_phase;

  if (millis() - last_ms > 10)
  {
    last_ms = millis();

    switch (measurement_phase)
    {
    case 0:
    {
      sonarLeft.startMeasurement();
      sonarLeft.update();
      measurement_phase++;
    }
    break;
    case 1:
    {
      sonarFront.startMeasurement();
      sonarFront.update();
      measurement_phase++;
    }
    break;
    case 2:
    {
      sonarRight.startMeasurement();
      sonarRight.update();
      measurement_phase++;
    }
    break;
    default:
    {
      measurement_phase = 0;
    }
    }
  }
}

/**
 * @brief Checks if there is a large gap to the left or the right, depending on the direction.
 * @return True if large horizontal distance is detected.
 */
bool gapDetected()
{
  if (direction == Direction::ANTI_CLOCKWISE && currentReadings.distance_left >= RaceConstants::MAX_DISTANCE)
  {
    return true;
  }
  else if (direction == Direction::CLOCKWISE && currentReadings.distance_right >= RaceConstants::MAX_DISTANCE)
  {
    return true;
  }
  else
  {
=======
}

/**
 * @brief Function to print the sensor readings on the LCD display.
 */
void lcdPrintValues() {
  static unsigned long last_ms;
  static bool setup_print_values;

  if (millis() - last_ms > 200) {
    last_ms = millis();

    if (!setup_print_values) {
      lcdPrintValueSetup();
      setup_print_values = !setup_print_values;
    }

    lcdUpdate(lastReadings.distance_left, currentReadings.distance_left, 1, 0, 2, false);
    lcdUpdate(lastReadings.distance_front, currentReadings.distance_front, 5, 0, 2, false);
    lcdUpdate(lastReadings.distance_right, currentReadings.distance_right, 9, 0, 2, false);
    lcdUpdate(lastReadings.yaw_angle, currentReadings.yaw_angle, 12, 0, 3, true);
    lcdUpdate(lastReadings.voltage, currentReadings.voltage, 14, 1, 2, false);
  }
}


///----------------------------------------
/// @subsubsection ACCESORS
///----------------------------------------

/**
 * @brief Checks if there is a large gap to the left or the right, depending on the direction.
 * @return True if large horizontal distance is detected.
*/
bool gapDetected() {
  if (direction == Direction::ANTI_CLOCKWISE && currentReadings.distance_left >= RaceDistances::MAX_DISTANCE) {
    return true;
  } else if (direction == Direction::CLOCKWISE && currentReadings.distance_right >= RaceDistances::MAX_DISTANCE) {
    return true;
  } else {
>>>>>>> 7156fcf (Large Update)
    return false;
  }
}

/**
 * @brief Gets the direction based on whether the gap is to the left or the right side.
<<<<<<< HEAD
 * @return The direction.
 */
Direction getDirection()
{
  if (currentReadings.distance_left >= RaceConstants::MAX_DISTANCE)
  {
    direction = Direction::ANTI_CLOCKWISE;
  }
  else
  {
    direction = Direction::CLOCKWISE;
  }
  return direction;
}

=======
 * @return True if the direction has been determined, false otherwise.
*/
bool getDirection() {
  if (currentReadings.distance_left >= RaceDistances::MAX_DISTANCE) {
    direction = Direction::ANTI_CLOCKWISE;
    return true;
  } else if (currentReadings.distance_right >= RaceDistances::MAX_DISTANCE) {
    direction = Direction::CLOCKWISE;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Gets the number of sections completed by the robot.
 * @return The amount of completed sections.
*/
uint8_t getSections() {
  return abs(currentReadings.yaw_angle) / 90;
}

/**
 * @brief Gets the number of laps completed by the robot.
 * @return The amount of completed rounds.
*/
uint8_t getLaps() {
  return abs(currentReadings.yaw_angle) / 360;
}

/**
 * @brief Checks whether the robot is driving the final section.
 * @return True if the robot will now go through the final section, false otherwise.
*/
bool isFinalSection() {
  if (getLaps() == 3) {
    return true;
  } else {
    return false;
  }
}


>>>>>>> 7156fcf (Large Update)
///----------------------------------------
/// @subsection MANOEUVRES
///----------------------------------------

/**
 * @brief Manoeuvre for turning left or right, depending on the direction.
<<<<<<< HEAD
 */
void turn()
{
  motor.setSpeed(50);
  switch (direction)
  {
  case Direction::ANTI_CLOCKWISE:
  {
    servo.write(SteeringAngles::MAX_LEFT);
  }
  break;
  case Direction::CLOCKWISE:
  {
    servo.write(SteeringAngles::MAX_RIGHT);
  }
  break;
  }
}

=======
*/
void turn() {
  motor.setSpeed(50);
  switch (direction) {
    case Direction::ANTI_CLOCKWISE:
      {
        servo.write(SteeringAngles::MAX_LEFT);
      }
      break;
    case Direction::CLOCKWISE:
      {
        servo.write(SteeringAngles::MAX_RIGHT);
      }
      break;
  }
}

/**
 * @brief Short routine for stopping the robot.
*/
void stop() {
  servo.write(SteeringAngles::STRAIGHT);
  motor.stop();
  lcdShutdown();
  lcd.noBacklight();
}


>>>>>>> 7156fcf (Large Update)
///----------------------------------------
/// @subsection CONTROL LOOPS
///----------------------------------------

/**
 * @brief Method which combines the  different types of control loops
  used in this program by linking each of them to different conditions.
*/
<<<<<<< HEAD
void autoSteering()
{
  if (gapDetected())
  {
    turn();
  }
  else
  {
    switch (direction)
    {
    case Direction::ANTI_CLOCKWISE:
    {
      if (currentReadings.distance_left <= 35 || currentReadings.distance_left >= 45)
      {
        directionDependentControlLoop();
      }
      else
      {
        gyroscopeControlLoop();
      }
    }
    break;
    case Direction::CLOCKWISE:
    {
      if (currentReadings.distance_right <= 35 || currentReadings.distance_right >= 45)
      {
        directionDependentControlLoop();
      }
      else
      {
        gyroscopeControlLoop();
      }
    }
    break;
    default:
    {
      gyroscopeControlLoop();
    }
=======
void autoSteering() {
  if (gapDetected()) {
    turn();
  } else {
    // Set the motor speed to default speed
    motor.setSpeed(80);

    // Choose fitting control loop depending on which conditions are fulfilled
    switch (direction) {
      case Direction::ANTI_CLOCKWISE:
        {
          if (isFinalSection()) {
            anticlockwiseSteering.setpoint(initialReadings.distance_left);
          }
          if (currentReadings.distance_left <= 35 || currentReadings.distance_left >= 45) {
            directionDependentControlLoop();
          } else {
            gyroscopeControlLoop();
          }
        }
        break;
      case Direction::CLOCKWISE:
        {
          if (isFinalSection()) {
            anticlockwiseSteering.setpoint(initialReadings.distance_left);
          }
          if (currentReadings.distance_right <= 35 || currentReadings.distance_right >= 45) {
            directionDependentControlLoop();
          } else {
            gyroscopeControlLoop();
          }
        }
        break;
      default:
        {
          gyroscopeControlLoop();
        }
>>>>>>> 7156fcf (Large Update)
    }
  }
}

/**
 * @brief Simple three state controller that makes the robot approximate
<<<<<<< HEAD
 the distance to a given setpoint value.
*/
void directionDependentControlLoop()
{
  int16_t input, output;

  // Set the speed of the robot
  motor.setSpeed(80);

  switch (direction)
  {
  case Direction::ANTI_CLOCKWISE:
  {
    input = currentReadings.distance_left;
    anticlockwiseSteering.update(input);
    output = anticlockwiseSteering.getOutput();
  }
  break;
  case Direction::CLOCKWISE:
  {
    input = currentReadings.distance_right;
    clockwiseSteering.update(input);
    output = clockwiseSteering.getOutput();
  }
  break;
  }

  // Set the steering angle to the computed output value
  servo.write(output);
}

/**
 * @brief Simple three state controller that makes the robot approximate
 the left distance to the right distance.
*/
void neutralControlLoop()
{
  int16_t input, output;

  // Set the speed of the robot
  motor.setSpeed(80);

=======
 the left distance to the right distance.
*/
void neutralControlLoop() {
  int16_t input, output;

>>>>>>> 7156fcf (Large Update)
  input = currentReadings.distance_left;
  neutralSteering.setpoint(currentReadings.distance_right);
  neutralSteering.update(input);
  output = neutralSteering.getOutput();

  // Set the steering angle to the computed output value
  servo.write(output);
}

/**
<<<<<<< HEAD
 * @brief Simple three state controller which tries to approximate the angular
 velocity to zero.
*/
void gyroscopeControlLoop()
{
  int16_t input, output;

  // Set the speed of the robot
  motor.setSpeed(80);

=======
 * @brief Simple three state controller that makes the robot approximate
 the distance to a given setpoint value.
*/
void directionDependentControlLoop() {
  int16_t input, output;

  switch (direction) {
    case Direction::ANTI_CLOCKWISE:
      {
        input = currentReadings.distance_left;
        anticlockwiseSteering.update(input);
        output = anticlockwiseSteering.getOutput();
      }
      break;
    case Direction::CLOCKWISE:
      {
        input = currentReadings.distance_right;
        clockwiseSteering.update(input);
        output = clockwiseSteering.getOutput();
      }
      break;
  }

  // Set the steering angle to the computed output value
  servo.write(output);
}

/**
 * @brief Simple three state controller which tries to approximate the angular
 velocity to zero.
*/
void gyroscopeControlLoop() {
  int16_t input, output;

>>>>>>> 7156fcf (Large Update)
  input = currentReadings.angular_velocity;
  gyroscopeSteering.update(input);
  output = gyroscopeSteering.getOutput();

  // Set the steering angle to the computed output value
  servo.write(output);
}

<<<<<<< HEAD
=======

>>>>>>> 7156fcf (Large Update)
/**
 * @brief Control loop that makes the robot drive straight in the centre
 of the parcour.
*/

<<<<<<< HEAD
void driveStraightControlLoop()
{
=======
void driveStraightControlLoop() {
>>>>>>> 7156fcf (Large Update)
  int16_t horizontal_distance, input, output;

  // Set the speed of the robot
  motor.setSpeed(70);

  horizontal_distance = currentReadings.distance_left + currentReadings.distance_right;
  input = map(currentReadings.distance_left, 0, horizontal_distance, 0, 180);
  driveStraightPID.update(input);
  output = driveStraightPID.getOutput();

  // Set the steering angle to the computed output value
  servo.write(output);

  // Print the raw input, output and setpoint values with Serial Plotter
  driveStraightPID.serialPlotGraph();
<<<<<<< HEAD
}

///----------------------------------------
/// @section LCD ENHANCEMENT
///----------------------------------------

/**
 * @brief Function to print the sensor readings on the LCD display.
 */
void lcdPrintValues()
{
  static unsigned long last_ms;
  static bool setup_print_values;

  if (millis() - last_ms > 200)
  {
    last_ms = millis();

    if (!setup_print_values)
    {
      lcdPrintValueSetup();
      setup_print_values = !setup_print_values;
    }

    lcdUpdate(lastReadings.distance_left, currentReadings.distance_left, 1, 0, 2, false);
    lcdUpdate(lastReadings.distance_front, currentReadings.distance_front, 5, 0, 2, false);
    lcdUpdate(lastReadings.distance_right, currentReadings.distance_right, 9, 0, 2, false);
    lcdUpdate(lastReadings.yaw_angle, currentReadings.yaw_angle, 12, 0, 3, true);
    lcdUpdate(lastReadings.voltage, currentReadings.voltage, 14, 1, 2, false);
  }
=======
>>>>>>> 7156fcf (Large Update)
}