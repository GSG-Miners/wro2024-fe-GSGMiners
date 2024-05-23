/**
 * @file controller_v2.1
 * @brief Comprehensive control system for autonomous navigation and race management.
 *
 * This file encapsulates the logic for an autonomous vehicle's navigation system, integrating
 * sensor inputs, actuator control, and decision-making algorithms. It includes structures for
 * organizing sensor data and race state, initialization of hardware components, and a main loop
 * that orchestrates the vehicle's behavior based on sensor readings and race progress. The code
 * leverages modular design principles, allowing for clear separation of functionality and ease of
 * maintenance. Updated mechanisms for turning and section counting enhance the vehicle's ability
 * to navigate complex courses and track its performance throughout a race.
 *
 * @author Maximilian Kautzsch
 * @date Updated on 4th May 2024
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include "UltrasonicSensor.h"
#include "Config.h"
#include "Button.h"
#include "Display.h"
#include "L298N.h"
#include "Controlling.h"
#include "Camera.h"

///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// @section    DEFINTIONS
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Structures
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @struct Parameters
 * @brief Struct to group all global parameter values.
 */
struct Parameters
{
  Colour colour;
  uint8_t voltage;
  uint8_t y_pos;
  int16_t angular_velocity;
  int16_t yaw_angle;
  uint16_t distance_left;
  uint16_t distance_front;
  uint16_t distance_right;
  uint16_t steering_angle;
  uint16_t x_pos;
};

/**
 * @struct Race
 * @brief Struct to group all race-specified global variables.
 */
struct Race
{
  Direction direction;
  bool enabled;
  bool turning;
  bool obstacles_included;
  bool parking_enabled;
  uint8_t sections;
  uint8_t laps;
  int16_t setpoint_yaw_angle;
};

static Race race;
static Parameters initial;
static Parameters last;
Parameters current;

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Objects
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

// Initialize the sensor and button objects
UltrasonicSensor sonarLeft(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT);
UltrasonicSensor sonarFront(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT);
UltrasonicSensor sonarRight(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT);
MPU6050 imu(Wire);
L298N motor(Pins::MOTOR_FORWARD_PIN, Pins::MOTOR_BACKWARD_PIN);
Servo servo;
Camera camera;
Display display;

// Initialize the controller
DoubleSetpointController controller(ControllerDirection::DIRECT);

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Setup
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Initializes the system's hardware and software components.
 *
 * This function sets up the serial communication, initializes all connected sensors and actuators,
 * attaches the servo motor, and sets initial values for the controller and race direction. It is
 * called once when the program starts.
 */

void setup()
{
  // Init communication protocols
  Serial.begin(9600);
  Wire.begin();

  // Init sensors
  imu.begin();
  sonarLeft.begin();
  sonarFront.begin();
  sonarRight.begin();
  if (race.obstacles_included)
    camera.begin();

  // Init actuators
  motor.begin();
  servo.attach(Pins::SERVO_PIN);

  // Init the lcd display
  lcd.init();
  lcd.backlight();
  display.bootup();

  // Init controller
  controller.begin();
  controller.setStates(85, 90, 110);
  controller.setHysteresis(1);

  // Init Race Parameters
  race.obstacles_included = Mode::OBSTACLES_INCLUDED;
  race.parking_enabled = Mode::PARKING_ENABLED;
  race.direction = Direction::NONE;
}

///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// @section    MAIN PROGRAM
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/**
 * @brief Main loop function executed repeatedly during the robot's operation.
 *
 * Continuously performs essential tasks such as updating sensor readings,
 * displaying current data, and executing the robot's autonomous control algorithm.
 * This function ensures that the robot responds dynamically to real-time sensor
 * inputs and navigates effectively based on the implemented control logic.
 */
void loop()
{
  // Obtain and show the current measurement values on the display.
  updateSensors();
  showData();

  // The general algorithm of the robot's autonomous control system.
  drive();
}

///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// @section    METHODS
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Algorithm
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Executes the robot's autonomous control algorithm.
 *
 * Implements a multi-layered decision-making process to control the robot's movement during a race.
 * The function evaluates the current state, including laps completed, potential collisions, and
 * obstacles, and responds with appropriate actions such as stopping, parking, avoiding collisions,
 * steering, or turning. This hierarchical structure ensures the robot navigates efficiently and safely.
 *
 * @note The algorithm unfolds in three layers:
 * 1 Parking maneuvers or stopping
 * 2 Collision avoidance
 * 3 Navigation (obstacle steering, gyro-based steering, turning)
 */
void drive()
{
  // ⎧
  // ⎪
  // ⎩ LAYER 1
  if (getLaps() >= 3)
  {
    if (!race.parking_enabled && current.distance_front <= initial.distance_front + Constants::STOPPING_OFFSET)
      stop();
    else if (race.obstacles_included && race.parking_enabled && detectedParkingLot())
      park();
  }
  else
  {
    // ⎪
    // ⎩ LAYER 2
    if (collisionRisk())
      avoidCollision();
    else
    {
      // ⎪
      // ⎩ LAYER 3
      int8_t error = race.setpoint_yaw_angle - current.yaw_angle;

      if (race.obstacles_included && pixy.ccc.numBlocks)
        obstacleSteering();
      else
        gyroSteering();

      if (error >= -1 && error <= 1)
        race.turning = false;

      if (current.distance_front < 80 && detectedGap() && !race.turning && !pixy.ccc.numBlocks)
        turn();
    }
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Initialization and Shutdown
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Captures initial measurement values to establish a baseline.
 *
 * Engages the IMU and front sonar to record the starting orientation and distance. This function
 * sets the initial conditions that future measurements will be compared against, ensuring accurate
 * relative readings throughout the robot's operation.
 */
void initGyroscope()
{
  static bool is_initialized;

  if (!is_initialized)
  {
    imu.calcGyroOffsets();
    imu.update();
    initial.yaw_angle = imu.getAngleZ();
    is_initialized = true;
  }
}

/**
 * @brief Captures initial measurement values of the front sonar to establish a baseline.
 *
 * Activates the front sonar to record the starting distance. This function sets the initial
 * condition for the front sonar that future measurements will be compared against, ensuring
 * accurate distance readings throughout the robot's operation.
 */
void initSonars()
{
  static bool is_initialized;

  if (!is_initialized && current.distance_front != 0 && current.distance_front != 400)
  {
    initial.distance_front = current.distance_front;
    is_initialized = true;
  }
}

/**
 * @brief Ceases all robot activities, transitioning to a safe, inactive state.
 *
 * Commands the robot to a full stop by setting the steering to a neutral position and powering down
 * the motor and servo. This function is critical for ensuring the robot's immediate cessation of
 * movement in response to external commands or internal conditions.
 */
void stop()
{
  setAngle(Constants::STRAIGHT);
  motor.end();
  servo.detach();
  race.enabled = false;
}

/**
 * @brief Manages the automated parking process of the vehicle.
 *
 * This function orchestrates the steps involved in automatically parking the vehicle.
 * It utilizes a state machine approach to transition through various phases of the parking
 * maneuver, including positioning, reversing, and steering adjustments. The function relies
 * on time intervals and sensor feedback to control the vehicle's movements and ensure it
 * safely enters the parking spot.
 */
void park()
{
  const uint8_t FREQUENCY = 10;
  const uint8_t PARKING_TIME = 1500;
  static uint8_t state;
  static unsigned long last_millis_A, last_millis_B;

  if (millis() - last_millis_A > (1000 / FREQUENCY))
  {
    last_millis_A = millis();

    switch (state)
    {
    case 0: // Approach parking spot.
    {
      motor.write(50);
      gyroSteering();

      // Check if the vehicle is close enough to the parking spot.
      if (current.distance_front < 80)
      {
        motor.stop();
        last_millis_B = last_millis_A;
        state++;
      }
    }
    break;
    case 1: // Prepare to reverse.
    {
      gyroSteering();

      // Wait for 3 seconds before reversing.
      if (millis() - last_millis_B > 3000)
      {
        race.setpoint_yaw_angle += 45;
        state++;
      }
    }
    break;
    case 2: // Begin reversing into the spot.
    {
      motor.write(-50);
      gyroSteering();

      // Check if the steering angle is correct for parking.
      if (current.steering_angle > 0)
      {
        last_millis_B = millis();
        state++;
      }
    }
    break;
    case 3: // Continue reversing with a set duration.
    {
      motor.write(-50);
      gyroSteering();

      // Check if the vehicle has reversed for the set parking duration.
      if (millis() - last_millis_B > PARKING_TIME)
      {
        race.setpoint_yaw_angle -= 45;
        state++;
      }
    }
    break;
    case 4: // Final adjustments.
    {
      motor.write(-50);
      gyroSteering();

      // Check if the vehicle is properly aligned within the spot.
      if (current.steering_angle > 0)
        stop();
    }
    break;
    }
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Sensor and Actuator Management
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Periodically refreshes sensor data to provide an accurate environmental model.
 *
 * Gathers and updates readings from the gyroscope, ultrasonic sensors, and voltage sensor at
 * regular intervals. This continuous refreshment of data is essential for the robot to make
 * informed decisions based on the latest information about its surroundings and internal state.
 */
void updateSensors()
{
  const uint8_t IMU_FREQUENCY = 100;
  const uint8_t CAMERA_FREQUENCY = 20;
  static unsigned long last_millis_A;
  static unsigned long last_millis_B;

  // Calibrate the gyroscope by measuring the initial offset.
  initGyroscope();

  // Update the data stream of the gyroscope and the voltmeter.
  if (millis() - last_millis_A > (1000 / IMU_FREQUENCY))
  {
    last_millis_A = millis();

    imu.update();
    current.angular_velocity = imu.getGyroZ();
    current.yaw_angle = imu.getAngleZ() - initial.yaw_angle;
    current.voltage = map(analogRead(Pins::VOLTAGE_MEASUREMENT_PIN), 0, 1023, 0, 100);
  }

  // Update the retrieved data of the pixy camera.
  if (millis() - last_millis_B > (1000 / CAMERA_FREQUENCY))
  {
    last_millis_B = millis();

    pixy.ccc.getBlocks();
    current.x_pos = camera.readX();
    current.y_pos = camera.readY();
    current.colour = camera.readColour();
  }

  // Refresh the incoming ultrasonic sensor data by cycling through each sonar.
  cycleSonars();
  sonarLeft.update();
  sonarFront.update();
  sonarRight.update();
  current.distance_left = sonarLeft.readDistance();
  current.distance_front = sonarFront.readDistance();
  current.distance_right = sonarRight.readDistance();

  // Measure the initial front distance of the robot.
  initSonars();
}

/**
 * @brief Re-evaluates and updates actuator states based on sensor feedback.
 *
 * Monitors changes in steering angle and updates actuator commands accordingly. This function
 * also tracks the robot's progress through the race course, incrementing section counts post-turn
 * and refreshing race-specific parameters as needed.
 */
void setAngle(uint8_t steering_angle)
{
  static uint8_t last_angle;

  // Transfer the steering angle to the servo, if changed.
  if (steering_angle != last_angle)
  {
    last_angle = steering_angle;
    servo.write(steering_angle);
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Navigation Control
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Manages the vehicle's lateral navigation during straight segments.
 *
 * Aligns the vehicle parallel to the track walls by adjusting the steering angle. The function
 * takes into account the race direction to optimize the wall-following behavior. It is a key
 * component in maintaining a steady course and preventing drift during straight-line travel.
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
    controller.setSetpoint(40);
    current.steering_angle = controller.getOutput(current.distance_right);
  }
}

/**
 * @brief Navigates the robot centrally along the straight path.
 *
 * Ensures the robot maintains a central trajectory by dynamically adjusting the steering angle
 * based on the distance from the walls. This method is integral to the robot's ability to
 * navigate straight corridors without veering off course.
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
 * @brief Maintains the robot's orientation by stabilizing angular velocity.
 *
 * Actively monitors and adjusts the robot's steering to counteract any unwanted rotational
 * movement. This method is crucial for ensuring the robot's orientation remains consistent,
 * particularly during maneuvers that could cause it to deviate from its intended heading.
 */
void gyroSteering()
{
  // Transfer the default speed to the motor
  motor.write(Constants::DEFAULT_SPEED);

  // Calculate and store the output of the control loop
  controller.setDirection(ControllerDirection::REVERSE);
  controller.setSetpoint(race.setpoint_yaw_angle);
  setAngle(controller.getOutput(current.yaw_angle));
}

/**
 * @brief Conducts the turn maneuver aligned with the race direction.
 *
 * Adjusts the robot's speed and steering angle to execute a turn. The function takes into account
 * the current race direction to determine the appropriate steering angle, facilitating a smooth
 * and controlled navigational change.
 */
void turn()
{
  // Transfer a lowered speed to the motor.
  motor.write(70);

  // Set the steering angle to the maximum of the corresponding direction.
  if (race.direction == Direction::ANTICLOCKWISE)
  {
    setAngle(Constants::MAX_LEFT);
    race.setpoint_yaw_angle += 90;
    race.turning = true;
  }
  else if (race.direction == Direction::CLOCKWISE)
  {
    setAngle(Constants::MAX_RIGHT);
    race.setpoint_yaw_angle -= 90;
    race.turning = true;
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Collision and Obstacle Management
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Evaluates the risk of collision based on sensor data.
 *
 * This function assesses the distances measured by the front and side ultrasonic sensors to
 * determine if the vehicle is at risk of colliding with an object. It is used to trigger
 * evasive maneuvers if necessary.
 *
 * @return True if a collision is imminent, false otherwise.
 */
bool collisionRisk()
{
  if (current.distance_left <= Constants::MIN_DISTANCE || current.distance_right <= Constants::MAX_DISTANCE)
    return true;
  else
    return false;
}

/**
 * @brief Implements collision avoidance by adjusting speed and steering.
 *
 * In the event of an imminent collision, this function reduces the vehicle's speed and sets the
 * steering angle to avoid the obstacle. It is a critical safety feature that prevents crashes
 * and allows the vehicle to continue racing.
 */
void avoidCollision()
{
  if (current.distance_left <= Constants::MIN_DISTANCE)
  {
    setAngle(110);
    motor.write(60);
  }
  if (current.distance_right <= Constants::MIN_DISTANCE)
  {
    setAngle(60);
    motor.write(60);
  }
}

/**
 * @brief Manages the robot's steering when encountering obstacles.
 *
 * Adjusts the robot's trajectory when approaching colored obstacles by modifying the steering angle.
 * This function is essential for the robot's ability to detect and respond to obstacles based on their color,
 * allowing for smooth navigation around them without collision.
 *
 * @note For red obstacles, reverse the direction and calculate a setpoint that is inversely proportional
 * to the y-position of the obstacle within the frame, scaled by the frame's dimensions.
 * For green obstacles, also reverse the direction but calculate a setpoint that is directly proportional
 * to the y-position of the obstacle, scaled similarly.
 * This results in a steering angle that guides the robot around the obstacle.
 */
void obstacleSteering()
{
  // Transfer a lowered speed to the motor to allow for precise maneuvering.
  motor.write(70);

  // Calculate and store the output of the controller, depending on the color of the block.
  // This conditional steering adjustment is based on the visual input from the sensor.
  switch (current.colour)
  {
  case Colour::RED:
  {
    controller.setDirection(ControllerDirection::REVERSE);
    controller.setSetpoint(-1 * ((current.y_pos - pixy.frameHeight) * pixy.frameHeight) / (2 * pixy.frameWidth));
    current.steering_angle = controller.getOutput(current.x_pos);
  }
  break;
  case Colour::GREEN:
  {
    controller.setDirection(ControllerDirection::REVERSE);
    controller.setSetpoint(((current.y_pos + pixy.frameHeight) * pixy.frameHeight) / (2 * pixy.frameWidth));
    current.steering_angle = controller.getOutput(current.x_pos);
  }
  break;
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Race Progress Monitoring
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Retrieves the number of sections completed by the vehicle.
 *
 * This function calculates the number of sections the vehicle has passed through in the current
 * lap. It is useful for tracking the vehicle's progress through the race course.
 *
 * @return The number of sections completed.
 */
uint8_t getSections()
{
  race.sections = abs(current.yaw_angle) / 87.5;
  return race.sections;
}

/**
 * @brief Computes the number of laps completed by the vehicle.
 *
 * This function divides the total number of sections completed by the number of sections per lap
 * to determine the total laps completed. It is essential for race management and determining the
 * winner.
 *
 * @return The number of laps completed.
 */
uint8_t getLaps()
{
  race.laps = abs(current.yaw_angle) / 350;
  return race.laps;
}

/**
 * @brief Detects significant gaps indicating potential turns in the race track.
 *
 * This function checks the distances on either side of the vehicle to identify large gaps that
 * could signify an upcoming turn or an open area. It influences the vehicle's steering decisions
 * and is critical for successful navigation.
 *
 * @return True if a large gap is detected, false otherwise.
 */
bool detectedGap()
{
  switch (race.direction)
  {
  case Direction::ANTICLOCKWISE:
  {
    if (current.distance_left >= Constants::MAX_DISTANCE)
      return true;
    else
      return false;
  }
  break;
  case Direction::CLOCKWISE:
  {
    if (current.distance_right >= Constants::MAX_DISTANCE)
      return true;
    else
      return false;
  }
  break;
  }
}

/**
 * @brief Determines the vehicle's race direction based on sensor data.
 *
 * This function analyzes the distances on the left and right sides of the vehicle to set the
 * race direction. It adjusts the setpoint angle and steering angle accordingly, which is pivotal
 * for the vehicle's turning behavior.
 */
bool getDirection()
{
  static bool direction_determined;

  // Make sure that the direction is only determined once.
  if (direction_determined)
    return false;

  if (current.distance_right >= Constants::MAX_DISTANCE)
  {
    race.direction = Direction::CLOCKWISE;
    direction_determined = true;
    return true;
  }
  else if (current.distance_left >= Constants::MAX_DISTANCE)
  {
    race.direction = Direction::ANTICLOCKWISE;
    direction_determined = true;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Determines if the robot has detected a parking lot based on color detection.
 *
 * Checks the current detected color to determine if the robot is over a parking lot.
 * The function returns true if the detected color is magenta, indicating the presence of
 * a parking lot. Otherwise, it returns false.
 *
 * @return True if a magenta color is detected, indicating a parking lot; otherwise, false.
 */
bool detectedParkingLot()
{
  static bool detected_parking_lot;

  if (current.colour == Colour::MAGENTA)
    detected_parking_lot = true;

  return detected_parking_lot;
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Sensor Data Processing
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Identifies significant, consecutive readings above a defined threshold.
 *
 * Analyzes input values to detect peaks, which are defined as consecutive readings above a
 * specified threshold. This detection is vital for interpreting sensor data that may influence
 * the robot's decision-making processes.
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
      matches++;

    if (matches >= consecutive_matches)
      return true;
  }
  else
  {
    matches = 0;
    return false;
  }
}

/**
 * @brief Cycles through sonar sensors to measure distances in a time-sliced manner.
 *
 * Orchestrates the timing of distance measurements from multiple sonar sensors to prevent
 * interference. This method ensures that each sensor can operate without conflict, providing
 * reliable distance data for navigation.
 */
void cycleSonars()
{
  const uint8_t FREQUENCY = 50;
  static uint8_t state;
  static unsigned long last_millis;

  if (millis() - (1000 / FREQUENCY) > last_millis)
  {
    last_millis = millis();

    switch (state)
    {
    case 0:
    {
      if (!sonarLeft.isUpdating())
      {
        sonarFront.startMeasurement();
        state++;
      }
    }
    break;
    case 1:
    {
      if (!sonarFront.isUpdating())
      {
        sonarRight.startMeasurement();
        state++;
      }
    }
    break;
    case 2:
    {
      if (!sonarRight.isUpdating())
      {
        sonarLeft.startMeasurement();
        state = 0;
      }
    }
    break;
    }
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Display and User Interface
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

/**
 * @brief Visualizes sensor data on the LCD display for real-time monitoring.
 *
 * Executes periodic updates to the LCD display, presenting the latest sensor readings in an
 * easily interpretable format. This function enhances the user's ability to monitor the robot's
 * status and environmental interactions.
 */
void showData()
{
  const uint8_t FREQUENCY = 5;
  const uint8_t LAYOUT_ID = race.obstacles_included ? 2 : 1;
  static unsigned long last_millis;

  if (millis() - last_millis > (1000 / FREQUENCY))
  {
    last_millis = millis();

    // Print the display preset
    display.preset(LAYOUT_ID);

    // Update the data on the display
    switch (LAYOUT_ID)
    {
    case 0:
    {
      display.update(last.distance_left, current.distance_left, 1, 0, 2, false);
      display.update(last.distance_front, current.distance_front, 5, 0, 2, false);
      display.update(last.distance_right, current.distance_right, 9, 0, 2, false);
      display.update(last.yaw_angle, current.yaw_angle, 12, 0, 4, false);
      display.update(last.x_pos, current.y_pos, 1, 1, 3, false);
      display.update(last.y_pos, current.y_pos, 6, 1, 3, false);
      display.update(uint8_t(last.colour), uint8_t(current.colour), 11, 1, 1, false);
      display.update(last.voltage, current.voltage, 14, 1, 2, false);
    }
    break;
    case 1:
    {
      display.update(last.distance_left, current.distance_left, 1, 0, 3, false);
      display.update(last.distance_front, current.distance_front, 6, 0, 3, false);
      display.update(last.distance_right, current.distance_right, 11, 0, 3, false);
      display.update(last.yaw_angle, current.yaw_angle, 1, 1, 4, true);
      display.update(last.voltage, current.voltage, 11, 1, 2, false);
    }
    break;
    case 2:
    {
      display.update(last.distance_left, current.distance_left, 1, 0, 3, false);
      display.update(last.distance_front, current.distance_front, 6, 0, 3, false);
      display.update(last.distance_right, current.distance_right, 11, 0, 3, false);
      display.update(last.yaw_angle, current.yaw_angle, 1, 1, 3, false);
      display.update(last.x_pos, current.x_pos, 6, 1, 3, false);
      display.update(last.y_pos, current.y_pos, 11, 1, 3, false);
    }
    break;
    }
  }
}