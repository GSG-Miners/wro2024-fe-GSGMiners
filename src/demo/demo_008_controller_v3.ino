/**
 * @file demo_008_controller_v3.ino
 * @brief This sketch is used to test out an initial three point controller.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 29th January 2024 by Maximilian Kautzsch,
 * Finnian Belger & Logan Weigoldt
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include "ultrasonic_sensor.h"
#include "button.h"
#include "lcd_display.h"
#include "dc_motor.h"
#include "pid_controller.h"

///----------------------------------------
/// @section CONFIG
///----------------------------------------

///----------------------------------------
/// @subsection ENUMS
///----------------------------------------

/**
 * @enum Pins
 * @brief Enum to hold the pin numbers for the various components.
 */
enum Pins : const pin_size_t
{
  kTriggerPinLeft = 4,
  kEchoPinLeft = 3,
  kTriggerPinFront = 9,
  kEchoPinFront = 8,
  kTriggerPinRight = 17,
  kEchoPinRight = 16,
  kVoltageMeasurementPin = 14,
  kButtonPin = 15,
  kMotorForwardPin = 5,
  kMotorBackwardPin = 6,
  kServoPin = 7,
};

/**
 * @enum SteeringAngles
 * @brief Enum to hold the max values of the steering angles.
 */
enum SteeringAngles : const uint8_t
{
  kMaxLeft = 47,
  kMaxRight = 118,
  kStraight = 90
};

/**
 * @enum RaceConstants
 * @brief Enum to hold race specific constant parameters.
 */
enum RaceConstants : const uint8_t
{
  kMinDistance = 5,
  kMaxDistance = 130
};

/**
 * @enum Direction
 * @enum Enum to store names for the directions.
 */
enum Direction : const bool
{
  kClockwise = false,
  kAntiClockwise = true
};

///----------------------------------------
/// @subsection STRUCTURES
///----------------------------------------

/**
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
} SensorReadings;

/**
 * @struct ControllerParameters
 * @brief Struct to store all attributes that are used to configure a PID controller.
 */
typedef struct
{
  float proportional_gain;       ///< Gain of proportional term
  float integral_gain;           ///< Gain of integral term
  float derivative_gain;         ///< Gain of derivative term
  ControllerDirection direction; ///< Direction of the controller (direct or reverse)
} ControllerParameters;

/**
 * @struct RaceParameters
 * @brief Struct to store all the relevant race parameters.
 */
typedef struct
{
  Direction direction; ///< The direction of the race.
} RaceParameters;

// Initialize struct instances
SensorReadings lastReadings;
SensorReadings currentReadings;
SensorReadings initialReadings;
ControllerParameters paramsDriveStraightPID = {3.00, 1.20, 0.50, ControllerDirection::kDirect};
RaceParameters raceParameters;

///----------------------------------------
/// @subsection OBJECTS
///----------------------------------------

// Initialize the sensor and button objects
Button button(Pins::kButtonPin);
UltrasonicSensor sonarLeft(Pins::kTriggerPinLeft, Pins::kEchoPinLeft, SonarMode::kManual);
UltrasonicSensor sonarFront(Pins::kTriggerPinFront, Pins::kEchoPinFront, SonarMode::kManual);
UltrasonicSensor sonarRight(Pins::kTriggerPinRight, Pins::kEchoPinRight, SonarMode::kManual);
MPU6050 imu(Wire);
Motor motor(Pins::kMotorForwardPin, Pins::kMotorBackwardPin);
Servo servo;
PIDController driveStraightPID(paramsDriveStraightPID.direction);

///----------------------------------------
/// @subsection SETUP
///----------------------------------------

/**
 * @brief Setup function for the Arduino sketch.
 */
void setup()
{
  Serial.begin(9600);
  Wire.begin();

  imu.begin();
  lcd.init();
  lcd.backlight();

  servo.attach(Pins::kServoPin);
  motor.init();
  motor.setAcceleration(100);

  driveStraightPID.setLimits(SteeringAngles::kMaxLeft, SteeringAngles::kMaxRight);
  driveStraightPID.tune(paramsDriveStraightPID.proportional_gain, paramsDriveStraightPID.integral_gain, paramsDriveStraightPID.derivative_gain);
  driveStraightPID.setpoint(90);

  lcd.setCursor(2, 0);
  lcd.print("PRESS BUTTON");
  lcd.setCursor(4, 1);
  lcd.print("TO START");
}

///----------------------------------------
/// @section MAIN PROGRAM
///----------------------------------------

/**
 * @brief Main loop function for the Arduino sketch.
 */
void loop()
{

  // Update all of the measured values
  button.update();
  imu.update();
  sonarMeasurementSequence();
  motor.update();

  // Assign the modified sensor readings to variables
  currentReadings.button_count = button.getCount();
  currentReadings.voltage = map(analogRead(Pins::kVoltageMeasurementPin), 0, 1023, 0, 100);
  currentReadings.yaw_angle = imu.getAngleZ() - initialReadings.yaw_angle;
  currentReadings.angular_velocity = imu.getGyroZ();
  currentReadings.distance_front = sonarFront.getDistance();
  currentReadings.distance_left = sonarLeft.getDistance();
  currentReadings.distance_right = sonarRight.getDistance();

  if (currentReadings.button_count == 1)
  {
    static bool bootup_phase, setup_direction;

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

      // Automatic steering of the robot
      if (!setup_direction)
      {
        if (getDirection())
        {
          autoSteering();
          setup_direction = !setup_direction;
        }
      }

      autoSteering();
    }
    break;
    }
  }
  else if (currentReadings.button_count == 2)
  {
    lcdShutdown();
    lcd.noBacklight();
    servo.write(SteeringAngles::kStraight);
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
 * @brief Gets the number of laps completed by the robot.
 * @return The amount of completed rounds.
 */
uint8_t getLaps()
{
  return abs(currentReadings.yaw_angle) / 350;
}

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
 * @brief Checks if there is a large gap to the left or the right.
 * @return True if large horizontal distance is detected.
 */
bool gapDetected()
{
  switch (raceParameters.direction)
  {
  case Direction::kAntiClockwise:
  {
    if (currentReadings.distance_left >= RaceConstants::kMaxDistance)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  break;
  case Direction::kClockwise:
  {
    if (currentReadings.distance_right >= RaceConstants::kMaxDistance)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  break;
  }
}

/**
 * @brief Gets the direction based on whether the gap is to the left or the right side.
 * @return The direction.
 */
bool getDirection()
{
  if (currentReadings.distance_left >= RaceConstants::kMaxDistance)
  {
    raceParameters.direction = Direction::kAntiClockwise;
    return true;
  }
  else if (currentReadings.distance_right >= RaceConstants::kMaxDistance)
  {
    raceParameters.direction = Direction::kClockwise;
    return true;
  }
  else
  {
    return false;
  }
}

///----------------------------------------
/// @subsection MANOEUVRES
///----------------------------------------

void autoSteering()
{
  static unsigned long last_ms;
  if (millis() - last_ms > 10)
  {
    if (gapDetected())
    {
      turn();
    }
    else if (getLaps() == 3)
    {
      motor.stop();
    }
    else
    {
      switch (raceParameters.direction)
      {
      case Direction::kAntiClockwise:
      {
        if (currentReadings.distance_left >= 45 || currentReadings.distance_left <= 35)
        {
          threePointControlLoopV1();
        }
        else
        {
          threePointControlLoopV3();
        }
      }
      break;
      case Direction::kClockwise:
      {
        if (currentReadings.distance_right >= 45 || currentReadings.distance_right <= 35)
        {
          threePointControlLoopV2();
        }
        else
        {
          threePointControlLoopV3();
        }
      }
      break;
      }
    }
  }
}

/**
 * @brief Manoeuvre for turning left or right, depending on the direction.
 */
void turn()
{
  motor.setSpeed(50);
  switch (raceParameters.direction)
  {
  case Direction::kAntiClockwise:
  {
    servo.write(SteeringAngles::kMaxLeft);
  }
  break;
  case Direction::kClockwise:
  {
    servo.write(SteeringAngles::kMaxRight);
  }
  break;
  }
}

///----------------------------------------
/// @subsection CONTROL LOOPS
///----------------------------------------

/**
 * @brief Simple three point controller that makes the robot approximate
 the distance to a given setpoint value.
*/
void threePointControlLoopV1()
{
  const uint8_t setpoint = 40;
  const uint8_t hysterese = 1;
  const uint8_t left_steering_angle = 85;
  const uint8_t right_steering_angle = 100;

  // Set the speed of the robot
  motor.setSpeed(80);

  // Adjust the steering angle according to the left distance
  if (currentReadings.distance_left <= setpoint - hysterese)
  {
    servo.write(right_steering_angle);
  }
  else if (currentReadings.distance_left >= setpoint + hysterese)
  {
    servo.write(left_steering_angle);
  }
}

void threePointControlLoopV2()
{
  const uint8_t setpoint = 45;
  const uint8_t hysterese = 1;
  const uint8_t left_steering_angle = 85;
  const uint8_t right_steering_angle = 100;

  // Set the speed of the robot
  motor.setSpeed(80);

  // Adjust the steering angle according to the left distance
  if (currentReadings.distance_right <= setpoint - hysterese)
  {
    servo.write(left_steering_angle);
  }
  else if (currentReadings.distance_right >= setpoint + hysterese)
  {
    servo.write(right_steering_angle);
  }
}

/**
 * @brief Tries to keep the yaw angle at initial angle.
 */
void threePointControlLoopV3()
{
  const uint8_t hysterese = 1;
  const uint8_t left_steering_angle = 85;
  const uint8_t right_steering_angle = 100;

  // Set the speed of the robot
  motor.setSpeed(80);

  // Adjust the steering angle according to the left distance
  if (currentReadings.angular_velocity <= hysterese && currentReadings.angular_velocity >= -hysterese)
  {
    servo.write(SteeringAngles::kStraight);
  }
  else if (currentReadings.angular_velocity > hysterese)
  {
    servo.write(right_steering_angle);
  }
  else if (currentReadings.angular_velocity < -hysterese)
  {
    servo.write(left_steering_angle);
  }
}

/**
 * @brief Control loop that makes the robot drive straight in the centre
 of the parcour.
*/
void driveStraightControlLoop()
{
  int16_t horizontal_distance, input, output;

  motor.setSpeed(70);

  horizontal_distance = currentReadings.distance_left + currentReadings.distance_right;
  input = map(currentReadings.distance_left, 0, horizontal_distance, 0, 180);
  driveStraightPID.update(input);
  output = driveStraightPID.getOutput();
  servo.write(output);

  driveStraightPID.serialPlotGraph();
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
    lcdUpdate(lastReadings.yaw_angle, currentReadings.yaw_angle, 12, 0, 4, false);
    lcdUpdate(lastReadings.voltage, currentReadings.voltage, 14, 1, 2, false);
  }
}