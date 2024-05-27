/**
 * @file controller_v2.2_alpha
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
 * @date Updated on 27th May 2024
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#include <math.h>
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

///==================================================
/// @section    DEFINTIONS
///==================================================

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
  TurnState turn_state;
  TurnMode turn_mode;
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
  camera.begin();

  // Init actuators
  motor.begin();
  servo.attach(Pins::SERVO_PIN);
  servo.write(Constants::STRAIGHT);

  // Init the lcd display
  lcd.init();
  lcd.backlight();
  display.bootup();

  // Init controller
  controller.begin();
  controller.setHysteresis(1);

  // Init Race Parameters
  race.obstacles_included = Mode::OBSTACLES_INCLUDED;
  race.parking_enabled = Mode::PARKING_ENABLED;
  race.direction = Direction::NONE;
}

///==================================================
/// @section    MAIN PROGRAM
///==================================================

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
  // fixServo();
  drive();
}

///==================================================
/// @section    METHODS
///==================================================

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
  // LAYER 1
  if (getLaps() >= 3 && current.distance_front <= initial.distance_front + Constants::STOPPING_OFFSET)
  {
    stop();
  }
  else
  {
    // LAYER 2
    if (collisionRisk() && current.colour != Colour::RED && current.colour != Colour::GREEN)
    {
      avoidCollision();
    }
    else
    {
      // LAYER 3
      race.obstacles_included ? handleTurning(TurnMode::NOTE) : handleTurning(TurnMode::DEFAULT);
    }
  }
}

/**
 * @brief Orchestrates the vehicle's turning maneuvers based on the current turn mode.
 *
 * This function manages the complex process of turning by evaluating the vehicle's current
 * yaw angle, the desired setpoint yaw angle, and the presence of any detected gaps in the track.
 * It adjusts the vehicle's steering angle and speed to navigate turns smoothly and efficiently.
 * The function operates in three main states:
 * 1 INITIATE,
 * 2 TURNING and
 * 3 COMPLETE,
 * transitioning between these states as the turn progresses.
 *
 * @param turn_mode The mode of turning to be executed, influencing the turning behavior.
 */
void handleTurning(TurnMode turn_mode)
{
  static bool updated_setpoint_angle;
  int16_t angle_difference;

  switch (race.turn_state)
  {
  case TurnState::INITIATE:
  {
    // Preparing to initiate a turn based on detected track conditions and vehicle orientation
    getDirection();

    angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

    // Handle the turning logic based on the selected turn mode
    switch (turn_mode)
    {
    case TurnMode::DEFAULT:
    {
      // The default turning mode prioritizes gap detection and front distance
      if (detectedGap() && current.distance_front <= 100)
      {
        if (!updated_setpoint_angle)
        {
          if (race.direction == Direction::ANTICLOCKWISE)
            race.setpoint_yaw_angle += 90;
          else if (race.direction == Direction::CLOCKWISE)
            race.setpoint_yaw_angle -= 90;
          updated_setpoint_angle = true;
        }
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        race.turn_state = TurnState::TURNING;
      }
      else
      {
        gyroSteering(angle_difference);
        motor.write(Constants::DEFAULT_SPEED);
      }
    }
    break;
    case TurnMode::NOTE:
    {
      // The note turning mode considers the absence of visual markers and tighter front distance constraints
      if (!pixy.ccc.numBlocks && detectedGap() && current.distance_front <= 90)
      {
        if (!updated_setpoint_angle)
        {
          if (race.direction == Direction::ANTICLOCKWISE)
            race.setpoint_yaw_angle += 90;
          else if (race.direction == Direction::CLOCKWISE)
            race.setpoint_yaw_angle -= 90;
          updated_setpoint_angle = true;
        }
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        if (race.direction == Direction::ANTICLOCKWISE)
        {
          gyroSteering(angle_difference - 90);
          motor.write(Constants::REDUCED_SPEED);
        }
        else if (race.direction == Direction::CLOCKWISE)
        {
          gyroSteering(angle_difference + 90);
          motor.write(Constants::REDUCED_SPEED);
        }

        if (current.distance_front <= 15)
        {
          race.turn_state = TurnState::TURNING;
        }
      }
      else if (race.obstacles_included && (current.colour == Colour::RED || current.colour == Colour::GREEN))
      {
        obstacleSteering(current.x_pos, current.y_pos, current.colour);
        motor.write(Constants::REDUCED_SPEED);
      }
      else
      {
        gyroSteering(angle_difference);
        motor.write(Constants::REDUCED_SPEED);
      }
    }
    break;
    case TurnMode::SWIFT:
    {
      // The swift turning mode is designed for quick, sharp turns with reduced front distance checks
      if (detectedGap() && current.distance_front <= 40)
      {
        if (race.direction == Direction::ANTICLOCKWISE)
        {
          current.steering_angle = Constants::MAX_LEFT;
        }
        else if (race.direction == Direction::CLOCKWISE)
        {
          current.steering_angle = Constants::MAX_RIGHT;
        }

        if (!updated_setpoint_angle)
        {
          if (race.direction == Direction::ANTICLOCKWISE)
            race.setpoint_yaw_angle += 90;
          else if (race.direction == Direction::CLOCKWISE)
            race.setpoint_yaw_angle -= 90;
          updated_setpoint_angle = true;
        }

        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
        if (updated_setpoint_angle && abs(angle_difference) <= 60)
        {
          race.turn_state = TurnState::TURNING;
        }
      }
      else if (race.obstacles_included && (current.colour == Colour::RED || current.colour == Colour::GREEN))
      {
        obstacleSteering(current.x_pos, current.y_pos, current.colour);
        motor.write(Constants::REDUCED_SPEED);
      }
      else if (current.distance_left <= 150 && current.distance_right <= 150)
      {
        centreSteering();
        motor.write(Constants::REDUCED_SPEED);
      }
      else
      {
        gyroSteering(angle_difference);
        motor.write(Constants::REDUCED_SPEED);
      }
    }
    break;
    }

    motor.write(Constants::REDUCED_SPEED);
    updateSteeringAngle();
  }
  break;
  case TurnState::TURNING:
  {
    // Executing the turn based on the updated steering angle and monitoring the yaw angle
    switch (turn_mode)
    {
    case TurnMode::DEFAULT:
    {
      if (race.direction == Direction::ANTICLOCKWISE)
      {
        current.steering_angle = Constants::MAX_LEFT;
      }
      else if (race.direction == Direction::CLOCKWISE)
      {
        current.steering_angle = Constants::MAX_RIGHT;
      }

      motor.write(Constants::DEFAULT_SPEED);
      updateSteeringAngle();

      angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
      if (angle_difference >= -10 && angle_difference <= 10)
      {
        race.turn_state = TurnState::COMPLETE;
      }
    }
    break;
    case TurnMode::NOTE:
    {
      if (race.direction == Direction::ANTICLOCKWISE)
      {
        current.steering_angle = Constants::MAX_RIGHT;
      }
      else if (race.direction == Direction::CLOCKWISE)
      {
        current.steering_angle = Constants::MAX_LEFT;
      }

      motor.write(-Constants::REDUCED_SPEED - 20);
      updateSteeringAngle();

      angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
      if (abs(angle_difference) <= 40)
      {
        race.turn_state = TurnState::COMPLETE;
      }
    }
    break;
    case TurnMode::SWIFT:
    {
      angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

      if (angle_difference >= -2 && angle_difference <= 2)
        current.steering_angle = 90;
      if (angle_difference < -2 && angle_difference >= -10)
        current.steering_angle = 80;
      if (angle_difference > 2 && angle_difference <= 10)
        current.steering_angle = 105;
      if (angle_difference < -10)
        current.steering_angle = 50;
      if (angle_difference > 10)
        current.steering_angle = 120;

      motor.write(-Constants::REDUCED_SPEED);
      updateSteeringAngle();

      if (abs(angle_difference) <= 10)
      {
        race.turn_state = TurnState::COMPLETE;
      }
    }
    break;
    }
  }
  break;
  case TurnState::COMPLETE:
  {
    // Finalizing the turn, resetting the steering angle, and preparing for the next turn
    current.steering_angle = 90;
    updateSteeringAngle();
    updated_setpoint_angle = false;
    race.turn_state = TurnState::INITIATE;
  }
  break;
  }
}

/// ––––––––––––––––––––––––––––––––––––––––––––––––––
///  @subsection Initialization and Shutdown
/// ––––––––––––––––––––––––––––––––––––––––––––––––––

void begin()
{
  static bool flag;

  if (!flag)
  {
    current.steering_angle = Constants::STRAIGHT;
    motor.setAcceleration(100);
    flag = true;
  }
}

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
  current.steering_angle = Constants::STRAIGHT;
  updateSteeringAngle();
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
      gyroSteering(90);

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
      gyroSteering(90);

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
      gyroSteering(90);

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
      gyroSteering(90);

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
      gyroSteering(90);

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
  if (race.obstacles_included && millis() - last_millis_B > (1000 / CAMERA_FREQUENCY))
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
void updateSteeringAngle()
{
  // Transfer the steering angle to the servo, if changed.
  if (current.steering_angle != last.steering_angle)
  {
    last.steering_angle = current.steering_angle;
    servo.write(current.steering_angle);
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
  motor.write(Constants::DEFAULT_SPEED);

  // Calculate and store the output of the control loop
  controller.setStates(80, 90, 110);
  if (race.direction == Direction::ANTICLOCKWISE)
  {
    leftWallSteering(30);
  }
  else if (race.direction == Direction::CLOCKWISE)
  {
    rightWallSteering(30);
  }
}

/**
 * @brief Adjusts the vehicle's steering to maintain a set distance from the left wall.
 *
 * This function is called when the vehicle is navigating a track in an anticlockwise direction.
 * It ensures that the vehicle maintains a consistent lateral distance from the left wall by
 * adjusting the steering angle. This is crucial for smooth cornering and avoiding collisions
 * with the wall during left turns.
 *
 * @param setpoint_distance The target distance from the left wall (in centimeters).
 */
void leftWallSteering(uint16_t setpoint_distance)
{
  // Check if the vehicle is within the acceptable range of the setpoint distance
  if (current.distance_left >= setpoint_distance - 1 && current.distance_left <= setpoint_distance + 1)
  {
    // If within range, keep the steering straight
    current.steering_angle = Constants::STRAIGHT;
  }
  else if (current.distance_left > setpoint_distance + 1)
  {
    // If too far from the wall, steer right
    current.steering_angle = 105;
  }
  else if (current.distance_left < setpoint_distance - 1)
  {
    // If too close to the wall, steer left
    current.steering_angle = 80;
  }
}

/**
 * @brief Adjusts the vehicle's steering to maintain a set distance from the right wall.
 *
 * This function is invoked when the vehicle is navigating the track in a clockwise direction.
 * It corrects the steering angle to keep a steady distance from the right wall, facilitating
 * effective right turns and preventing the vehicle from scraping against the wall.
 *
 * @param setpoint_distance The desired distance to maintain from the right wall (in centimeters).
 */
uint16_t rightWallSteering(uint16_t setpoint_distance)
{
  // Evaluate if the vehicle's current distance from the right wall is within the setpoint range
  if (current.distance_right >= setpoint_distance - 1 && current.distance_right <= setpoint_distance + 1)
  {
    // Maintain a straight steering angle if within the target range
    current.steering_angle = Constants::STRAIGHT;
  }
  else if (current.distance_right > setpoint_distance + 1)
  {
    // Steer left if the vehicle is too far from the right wall
    current.steering_angle = 80;
  }
  else if (current.distance_right < setpoint_distance - 1)
  {
    // Steer right if the vehicle is too close to the right wall
    current.steering_angle = 105;
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
  // Calculate and store the output of the control loop
  /*
  if (current.distance_right >= current.distance_left - 1 && current.distance_right <= current.distance_left + 1) {
    current.steering_angle = Constants::STRAIGHT;
  } else if (current.distance_right < current.distance_left - 1) {
    current.steering_angle = 80;
  } else if (current.distance_right > current.distance_left + 1) {
    current.steering_angle = 105;
  }
  */

  current.steering_angle = constrain(0.10 * (current.distance_right - current.distance_left) + 90, Constants::MAX_LEFT, Constants::MAX_RIGHT);
}

/**
 * @brief Maintains the robot's orientation by stabilizing angular velocity.
 *
 * Actively monitors and adjusts the robot's steering to counteract any unwanted rotational
 * movement. This method is crucial for ensuring the robot's orientation remains consistent,
 * particularly during maneuvers that could cause it to deviate from its intended heading.
 */
void gyroSteering(int16_t angle_difference)
{
  // Calculate and store the output of the control loop
  if (angle_difference >= -2 && angle_difference <= 2)
    current.steering_angle = 90;
  if (angle_difference < -2 && angle_difference >= -10)
    current.steering_angle = 105;
  if (angle_difference > 2 && angle_difference <= 10)
    current.steering_angle = 80;
  if (angle_difference < -10)
    current.steering_angle = 115;
  if (angle_difference > 10)
    current.steering_angle = 70;
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
  if (current.distance_left <= Constants::MIN_DISTANCE || current.distance_right <= Constants::MIN_DISTANCE)
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
    current.steering_angle = 115;
  }
  if (current.distance_right <= Constants::MIN_DISTANCE)
  {
    current.steering_angle = 60;
  }

  updateSteeringAngle();
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
void obstacleSteering(uint16_t x, uint8_t y, Colour colour)
{
  // Transfer a lowered speed to the motor to allow for precise maneuvering.
  motor.write(Constants::REDUCED_SPEED);

  // Define the proportional gain constant.
  const float Kp = 0.75; // Adjust this value based on your system's requirements.

  // Calculate the setpoint based on the colour of the obstacle.
  float setpoint = 0.0;
  if (colour == Colour::RED)
  {
    // setpoint = (y - 207) / -3.000;
    setpoint = 26;
  }
  else if (colour == Colour::GREEN)
  {
    // setpoint = (y + 207) / 1.000;
    setpoint = 288;
  }
  else
  {
    // For other colours, maintain a neutral steering angle.
    current.steering_angle = 90;
    return;
  }

  // Calculate the error between the current position and the setpoint.
  float error = x - setpoint;

  // Compute the control output using the P-Controller.
  float control_output = Kp * error;

  // Apply the control output to adjust the steering angle.
  // Ensure the steering angle remains within the valid range.
  current.steering_angle = constrain(90 + control_output, Constants::MAX_LEFT, Constants::MAX_RIGHT);
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
  race.laps = abs(current.yaw_angle) / 348;
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
    {
      return true;
    }
    else
      return false;
  }
  break;
  case Direction::CLOCKWISE:
  {
    if (current.distance_right >= Constants::MAX_DISTANCE)
    {
      return true;
    }
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
  if (!direction_determined)
  {
    if (current.distance_left >= 180 && current.distance_left != 400 && current.distance_left > current.distance_right)
    {
      race.direction = Direction::ANTICLOCKWISE;
      direction_determined = true;
      return true;
    }
    else if (current.distance_right >= 180 && current.distance_right != 400 && current.distance_right > current.distance_left)
    {
      race.direction = Direction::CLOCKWISE;
      direction_determined = true;
      return true;
    }
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
  const uint8_t FREQUENCY = 20;
  static uint8_t state;
  static unsigned long last_millis;

  switch (state)
  {
  case 0:
  {
    if (!sonarLeft.isUpdating())
    {
      sonarFront.startMeasurement();
      if (millis() - last_millis > (1000) / FREQUENCY)
      {
        last_millis = millis();
        state++;
      }
    }
  }
  break;
  case 1:
  {
    if (!sonarFront.isUpdating())
    {
      sonarRight.startMeasurement();
      if (millis() - last_millis > (1000) / FREQUENCY)
      {
        last_millis = millis();
        state++;
      }
    }
  }
  break;
  case 2:
  {
    if (!sonarRight.isUpdating())
    {
      sonarLeft.startMeasurement();
      if (millis() - last_millis > (1000) / FREQUENCY)
      {
        last_millis = millis();
        state = 0;
      }
    }
  }
  break;
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
      display.update(0, (race.setpoint_yaw_angle - current.yaw_angle), 1, 1, 4, true);
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