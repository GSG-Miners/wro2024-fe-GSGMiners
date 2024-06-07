/**
 * @file controller_v2.3
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
 * @date Updated on 3rd June 2024
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#include <math.h>
#include <Servo.h>
#include "UltrasonicSensor.h"
#include "Gyroscope.h"
#include "Config.h"
#include "Button.h"
#include "Display.h"
#include "L298N.h"
#include "Controlling.h"
#include "Camera.h"

///==================================================
/// @section    DEFINTIONS
///==================================================

///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Structures
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @struct Parameters
 * @brief Struct to group all global parameter values.
 */
struct Parameters {
  Colour colour;
  int8_t speed;
  uint8_t voltage;
  uint8_t y_pos;
  uint8_t block_index;
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
struct Race {
  Direction direction;
  TurnMode turn_mode;
  bool enabled;
  bool turning;
  bool collision_avoidance_blocked;
  bool obstacle_steering_blocked;
  bool first_obstacle_detected;
  bool magenta_unlocked;
  bool obstacles_included;
  bool parking_enabled;
  uint8_t sections;
  uint8_t laps;
  int16_t setpoint_yaw_angle;
  int16_t drift_correction;
};

static Race race;
static Parameters initial;
static Parameters last;
Parameters current;

///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Objects
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

// Initialize the sensor and button objects
UltrasonicSensor sonarLeft(Pins::TRIGGER_PIN_LEFT, Pins::ECHO_PIN_LEFT);
UltrasonicSensor sonarFront(Pins::TRIGGER_PIN_FRONT, Pins::ECHO_PIN_FRONT);
UltrasonicSensor sonarRight(Pins::TRIGGER_PIN_RIGHT, Pins::ECHO_PIN_RIGHT);
L298N motor(Pins::MOTOR_FORWARD_PIN, Pins::MOTOR_BACKWARD_PIN);
Button button(Pins::BUTTON_PIN);
Servo servo;
Camera camera;
Gyroscope gyro;
Display display;

// Initialize the controller
DoubleSetpointController controller(ControllerDirection::DIRECT);

///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Setup
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Initializes the system's hardware and software components.
 *
 * This function sets up the serial communication, initializes all connected sensors and actuators,
 * attaches the servo motor, and sets initial values for the controller and race direction. It is
 * called once when the program starts.
 */

void setup() {
  // Init communication protocols
  Serial.begin(9600);

  // Init sensors
  gyro.begin();
  sonarLeft.begin();
  sonarFront.begin();
  sonarRight.begin();
  camera.begin();
  button.begin();
  pinMode(Pins::RELAY_PIN, OUTPUT);

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
void loop() {
  // Obtain and show the current measurement values on the display.
  updateSensors();
  showData();

  // The general algorithm of the robot's autonomous control system.
  (race.parking_enabled && race.obstacles_included) ? parkingDemo() : drive();
}


///==================================================
/// @section    METHODS
///==================================================


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Algorithm
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

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
void drive() {
  const uint8_t FREQUENCY = 20;
  static unsigned long last_millis;

  if (millis() - last_millis > (1000 / FREQUENCY)) {
    last_millis = millis();

    // LAYER 1
    if (getLaps() >= 3) {
      handleStop(race.obstacles_included);
    } else {
      // LAYER 2
      if (collisionRisk()) {
        avoidCollision();
      } else {
        // LAYER 3
        race.obstacles_included ? handleTurns(TurnMode::SWIFT) : handleTurns(TurnMode::SHARP);
      }
    }
  }

  motor.write(current.speed);
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Low-Layer-Logic (Triple-L)
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Handles the stopping behavior of the robot, considering obstacle presence.
 * 
 * This function manages the stopping process of the robot based on whether obstacles
 * are included in the race. It implements a three-layer decision-making process to
 * ensure safe and effective stopping or turning actions. The layers include checking
 * for stopping conditions, avoiding collisions, and handling turns.
 * 
 * @param obstacles_included A boolean flag indicating whether obstacles are included
 * in the race scenario.
 */
void handleStop(bool obstacles_included) {
  switch (obstacles_included) {
    case true:
      {
        // CONSTANT PARAMETERS.
        const uint16_t LAST_SECTION_DURATION = 3200;

        // Static variables for managing the stopping timer.
        static bool stopping_timer_locked;
        static unsigned long stopping_timer;

        // Check if the robot has completed 12 sections and start the stopping timer.
        if (getSections() >= 12 && !stopping_timer_locked) {
          stopping_timer = millis();
          stopping_timer_locked = true;
        }

        // Stop the robot if the front distance condition and timer criteria are met.
        if (current.distance_front <= initial.distance_front + Constants::STOPPING_OFFSET
            && stopping_timer_locked && millis() - stopping_timer > LAST_SECTION_DURATION) {
          stop();
        } else {
          // LAYER 2: Collision avoidance layer.
          if (collisionRisk() && !race.collision_avoidance_blocked) {
            avoidCollision();
          } else {
            // LAYER 3: Handle swift turns when obstacles are included.
            handleTurns(TurnMode::SWIFT);
          }
        }
      }
      break;
    case false:
      {
        // Stop the robot if the front distance condition is met.
        if (current.distance_front <= initial.distance_front + Constants::STOPPING_OFFSET) {
          stop();
        } else {
          // LAYER 2: Collision avoidance layer.
          if (collisionRisk() && !race.collision_avoidance_blocked) {
            avoidCollision();
          } else {
            // LAYER 3: Handle sharp turns when obstacles are not included.
            handleTurns(TurnMode::SHARP);
          }
        }
      }
      break;
  }
}

const uint16_t INTER_TURN_DURATION = 3000;
static bool inter_turn_timer_locked = true;
static long inter_turn_timer = -INTER_TURN_DURATION + 2000;

/**
 * @brief Manages the vehicle's turning behavior based on the specified turn mode.
 *
 * This function selects and executes the appropriate turning strategy according to the provided
 * turn mode. It allows the vehicle to adapt its turning behavior dynamically based on different
 * scenarios, such as sharp turns, adaptive turns, or swift turns for precise maneuvering.
 *
 * @param turn_mode The mode that determines which turn handling strategy to execute. It can be
 * TurnMode::SHARP, TurnMode::ADAPTIVE, or TurnMode::SWIFT.
 */
void handleTurns(TurnMode turn_mode) {
  // Preparing to initiate a turn based on detected track conditions and vehicle orientation.
  getDirection();

  switch (turn_mode) {
    case TurnMode::SHARP:
      {
        const uint8_t INITIATING_DISTANCE = 60;  // Distance at which the turning process is started.

        // Handles standard turning behavior.
        if ((detectedGap() && current.distance_front <= INITIATING_DISTANCE) || race.turning) {
          sharpTurn();
        } else {
          maintainStraightPath(race.setpoint_yaw_angle - current.yaw_angle);
          if (abs(current.yaw_angle) >= 980) current.speed = 60;
          else current.speed = Constants::DEFAULT_SPEED;
        }
        updateSteeringAngle();
      }
      break;
    case TurnMode::SWIFT:
      {
        int16_t angle_difference;
        const uint8_t INITIAL_MIN_DISTANCE = 0;
        const uint8_t INITIAL_MAX_DISTANCE = 60;

        // Boolean flags for various conditions.
        bool distance_in_range = (current.distance_front <= INITIAL_MAX_DISTANCE) ? 1 : 0;
        bool angle_in_range = (abs(race.setpoint_yaw_angle - current.yaw_angle) <= 20) ? 1 : 0;
        bool obstacles_detected = (current.colour == Colour::RED || current.colour == Colour::GREEN) ? 1 : 0;
        bool inter_turn_timer_passed = (inter_turn_timer_locked && millis() - inter_turn_timer > INTER_TURN_DURATION) ? 1 : 0;
        bool large_outer_distance = ((race.direction == Direction::ANTICLOCKWISE && current.distance_right >= 60) || (race.direction == Direction::CLOCKWISE && current.distance_left >= 60)) ? 1 : 0;

        // Handles swift turning behavior for more precise maneuvers.
        if ((detectedGap() && distance_in_range && angle_in_range && inter_turn_timer_passed) || race.turning) {
          swiftTurn();
        } else {
          // If first obstacle has been detected, save its index to the initial index.
          if (!race.first_obstacle_detected && pixy.ccc.numBlocks) {
            last.block_index = current.block_index;
            race.first_obstacle_detected = true;
          }

          // If the index has changed, block obstacle steering.
          if (current.block_index != last.block_index && race.first_obstacle_detected) {
            last.block_index = current.block_index;
            race.obstacle_steering_blocked = true;
          }

          // If conditions are not met, maintain the current path and manage obstacles.
          angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
          if (race.obstacles_included && pixy.ccc.numBlocks && !race.obstacle_steering_blocked) {
            race.collision_avoidance_blocked = false;
            obstacleSteering(current.x_pos, current.y_pos, current.colour);

            current.speed = Constants::REDUCED_SPEED;
          } else if (large_outer_distance) {
            race.collision_avoidance_blocked = false;
            trackOuterWall(60);
            current.speed = Constants::REDUCED_SPEED;
          } else {
            race.collision_avoidance_blocked = false;
            maintainStraightPath(angle_difference);
            current.speed = Constants::REDUCED_SPEED;
          }

          updateSteeringAngle();
        }
      }
      break;
  }
}

/**
 * @brief Manages the swift turning behavior of the robot.
 *
 * This function controls the turning process of the robot through a state machine approach,
 * ensuring rapid and efficient handling of turns based on the detected track conditions
 * and the vehicle's current orientation. It consists of four states: INITIATE, TURNING,
 * CORRECTING, and COMPLETE, to handle the entire turning process from start to finish.
 */
void swiftTurn() {
  // Enum for handling different turning states.
  enum class TurnState : const uint8_t {
    INITIATE,    // Initial state where the turn is prepared based on track conditions.
    TURNING,     // State where the robot is actively turning.
    CORRECTING,  // State for fine-tuning the turn angle.
    COMPLETE     // State indicating the turn has been completed.
  };

  // CONSTANT PARAMETERS.
  const uint8_t CORRECTING_ANGLE_DIFFERENCE = 55;
  const uint16_t REVERSE_DURATION = 4000;

  // Static variables for maintaining the current state and other control parameters.
  static TurnState turn_state;
  int16_t angle_difference;
  static bool updated_setpoint_angle;

  static bool turning_timer_locked;
  static unsigned long turning_timer;

  switch (turn_state) {
    case TurnState::INITIATE:
      {
        // Lock the turning process.
        inter_turn_timer_locked = false;
        race.turning = true;

        // Calculate the angle difference between the setpoint and current yaw angle.
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        // Calculate the angle difference between the setpoint and current yaw angle.
        race.collision_avoidance_blocked = false;

        // Update the setpoint yaw angle based on the turning direction.
        if (!updated_setpoint_angle) {
          if (race.direction == Direction::ANTICLOCKWISE) race.setpoint_yaw_angle += 90;
          else if (race.direction == Direction::CLOCKWISE) race.setpoint_yaw_angle -= 90;
          updated_setpoint_angle = true;
        }
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        turn_state = TurnState::TURNING;  // Transition to TURNING state.
      }
      break;
    case TurnState::TURNING:
      {
        bool small_outer_distance;

        if (race.direction == Direction::ANTICLOCKWISE && current.distance_right <= 50) small_outer_distance = true;
        else if (race.direction == Direction::CLOCKWISE && current.distance_left <= 50) small_outer_distance = true;
        else small_outer_distance = false;

        // Actively managing the turning process.
        race.collision_avoidance_blocked = true;  // Block collision avoidance during turn.

        // Lock the turning timer if not already locked.
        if (!turning_timer_locked) {
          turning_timer = millis();
          turning_timer_locked = true;
        }

        switch (small_outer_distance) {
          case true:
            {
              // Calculate the angle difference between the setpoint and current yaw angle.
              angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
              if ((abs(angle_difference) <= CORRECTING_ANGLE_DIFFERENCE) || (turning_timer_locked && millis() - turning_timer > 2500) && current.distance_front <= 10) {
                turn_state = TurnState::CORRECTING;  // Transition to CORRECTING state.
              } else {
                // Adjust steering angle based on the direction of the turn.
                if (race.direction == Direction::ANTICLOCKWISE) {
                  current.steering_angle = Constants::MAX_LEFT;
                } else if (race.direction == Direction::CLOCKWISE) {
                  current.steering_angle = Constants::MAX_RIGHT;
                }
              }
            }
            break;
          case false:
            {
              if (current.distance_front <= 5) {
                turn_state = TurnState::CORRECTING;  // Transition to CORRECTING state.
              } else {
                angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
                if (race.direction == Direction::ANTICLOCKWISE) {
                  maintainStraightPath(angle_difference - 90);
                } else if (race.direction == Direction::CLOCKWISE) {
                  maintainStraightPath(angle_difference + 90);
                }
              }
            }
            break;
        }

        current.speed = Constants::REDUCED_SPEED;  // Move the motor in reverse at reduced speed.
        updateSteeringAngle();
      }
      break;
    case TurnState::CORRECTING:
      {
        // Fine-tuning the turn angle to correct the robot's orientation.
        static unsigned long reversing_timer;
        static bool reversing_timer_locked;

        bool reverse_duration_passed = (reversing_timer_locked && millis() - reversing_timer > REVERSE_DURATION) ? 1 : 0;

        race.collision_avoidance_blocked = true;  // Unblock collision avoidance.

        // Calculate the angle difference between the setpoint and current yaw angle.
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        // Transition to COMPLETE state after reversing for the specified duration.
        if (reverse_duration_passed || abs(angle_difference) <= 20) {
          reversing_timer_locked = false;
          turn_state = TurnState::COMPLETE;  // Transition to COMPLETE state.
        } else {
          // Lock the reversing timer if not already locked.
          if (!reversing_timer_locked) {
            reversing_timer = millis();
            reversing_timer_locked = true;
          }

          if (angle_difference >= -10 && angle_difference <= 10) current.steering_angle = 90;
          if (angle_difference < -10 && angle_difference >= -40) current.steering_angle = 90;
          if (angle_difference > 10 && angle_difference <= 40) current.steering_angle = 90;
          if (angle_difference < -40) current.steering_angle = Constants::MAX_LEFT;
          if (angle_difference > 40) current.steering_angle = Constants::MAX_RIGHT;
        }

        current.speed = -Constants::REDUCED_SPEED - 10;  // Move the motor forward at reduced speed.
        updateSteeringAngle();
      }
      break;
    case TurnState::COMPLETE:
      {
        // Finalizing the turn and preparing for the next turn.
        if (!inter_turn_timer_locked) {
          inter_turn_timer = millis();
          inter_turn_timer_locked = true;
        }
        race.sections++;
        turning_timer_locked = false;
        updated_setpoint_angle = false;

        // Unlock the turning process.
        turn_state = TurnState::INITIATE;
        race.first_obstacle_detected = false;
        race.obstacle_steering_blocked = false;
        race.turning = false;
      }
      break;
  }
}

/**
 * @brief Manages the default turning behavior of the robot.
 * 
 * This function controls the turning process of the robot through a state machine approach,
 * ensuring smooth and efficient handling of turns based on the detected track conditions
 * and the vehicle's current orientation. It consists of three states: INITIATE, TURNING,
 * and COMPLETE, to handle the entire turning process from start to finish.
 */
void sharpTurn() {
  // Enum for handling different turning states.
  enum class TurnState : const uint8_t {
    INITIATE,  // Initial state where the turn is prepared based on track conditions.
    TURNING,   // State where the robot is actively turning.
    COMPLETE   // State indicating the turn has been completed.
  };

  const uint8_t COMPLETE_ANGLE_DIFFERENCE = 10;  // Angle difference indicating that the process is finished.

  static TurnState turn_state;
  static bool updated_setpoint_angle;
  int16_t angle_difference;

  switch (turn_state) {
    case TurnState::INITIATE:
      {
        // Lock the turning process.
        race.turning = true;

        // The default turning mode prioritizes gap detection and front distance.
        if (!updated_setpoint_angle) {
          if (race.direction == Direction::ANTICLOCKWISE) race.setpoint_yaw_angle += 90;
          else if (race.direction == Direction::CLOCKWISE) race.setpoint_yaw_angle -= 90;
          updated_setpoint_angle = true;
        }
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        turn_state = TurnState::TURNING;  // Transition to TURNING state.
      }
      break;
    case TurnState::TURNING:
      {
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
        if (abs(angle_difference) <= COMPLETE_ANGLE_DIFFERENCE) {
          turn_state = TurnState::COMPLETE;  // Transition to COMPLETE state.
        } else if (abs(angle_difference) <= COMPLETE_ANGLE_DIFFERENCE + 20) {
          maintainStraightPath(angle_difference);
        } else {
          // Adjust steering angle based on the direction of the turn.
          if (race.direction == Direction::ANTICLOCKWISE) {
            current.steering_angle = Constants::MAX_LEFT;
          } else if (race.direction == Direction::CLOCKWISE) {
            current.steering_angle = Constants::MAX_RIGHT;
          }
        }

        if (abs(current.yaw_angle) >= 980) current.speed = 60;
        else current.speed = Constants::DEFAULT_SPEED;
        updateSteeringAngle();
      }
      break;
    case TurnState::COMPLETE:
      {
        current.steering_angle = 90;
        updateSteeringAngle();
        race.sections++;
        updated_setpoint_angle = false;

        // Unlock the turning process.
        turn_state = TurnState::INITIATE;
        race.turning = false;
        return;
      }
      break;
  }
}

/**
 * @brief Manages the adaptive turning behavior of the robot.
 * 
 * This function controls the adaptive turning process of the robot through a state machine approach,
 * ensuring flexible and efficient handling of turns based on the detected track conditions and the
 * vehicle's current orientation. It consists of five states: INITIATE, TRACKING, TURNING, CORRECTING,
 * and COMPLETE, to manage the entire turning process from start to finish.
 */
void handleAdaptiveTurns() {
  // Enum for handling different turning states.
  enum class TurnState : const uint8_t {
    INITIATE,    // Initial state where the turn is prepared based on track conditions.
    TRACKING,    // State where the robot tracks the outer walls for better positioning.
    TURNING,     // State where the robot is actively turning.
    CORRECTING,  // State for fine-tuning the turn angle.
    COMPLETE     // State indicating the turn has been completed.
  };

  static TurnState turn_state;         // Variable to track the current state.
  static bool updated_setpoint_angle;  // Flag to indicate if the setpoint angle has been updated.
  int16_t angle_difference;            // Difference between the setpoint and current yaw angle.

  switch (turn_state) {
    case TurnState::INITIATE:
      {
        // Preparing to initiate a turn based on detected track conditions and vehicle orientation.
        getDirection();

        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
        if (detectedGap() && current.distance_front <= 70 && abs(angle_difference) <= 20) {
          race.collision_avoidance_blocked = false;
          angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

          turn_state = TurnState::TRACKING;  // Transition to the TRACKING state.
        } else {
          if (race.obstacles_included && pixy.ccc.numBlocks) {
            race.collision_avoidance_blocked = false;
            obstacleSteering(current.x_pos, current.y_pos, current.colour);

            current.speed = Constants::REDUCED_SPEED;
          } else {
            race.collision_avoidance_blocked = false;
            maintainStraightPath(angle_difference);
            current.speed = Constants::REDUCED_SPEED;
          }
        }

        updateSteeringAngle();
      }
      break;
    case TurnState::TRACKING:
      {
        // Tracking the outer walls to maintain optimal positioning.
        race.collision_avoidance_blocked = true;
        if (detectedGap() && current.distance_front <= 10 && abs(angle_difference) <= 20) {
          if (!updated_setpoint_angle) {
            // Update the setpoint angle based on the turn direction.
            if (race.direction == Direction::ANTICLOCKWISE) race.setpoint_yaw_angle += 90;
            else if (race.direction == Direction::CLOCKWISE) race.setpoint_yaw_angle -= 90;
            updated_setpoint_angle = true;
          } else {
            turn_state = TurnState::TURNING;  // Transition to TURNING state.
          }
        } else {
          // If not transitioning, maintain a straight path or track the outer wall.
          if (angle_difference <= -15 || angle_difference >= 15) {
            maintainStraightPath(angle_difference);
          } else {
            trackOuterWall(50);
          }
        }
        updateSteeringAngle();
        current.speed = Constants::REDUCED_SPEED;
      }
      break;
    case TurnState::TURNING:
      {
        // Actively turning the robot based on the setpoint angle.
        race.collision_avoidance_blocked = true;  // Block collision avoidance during turn.

        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
        if (abs(angle_difference) <= 30) {
          turn_state = TurnState::CORRECTING;  // Transition to CORRECTING state.
        } else if (current.colour == Colour::RED || current.colour == Colour::GREEN) {
          turn_state = TurnState::CORRECTING;  // Transition to CORRECTING state if certain colors are detected.
        } else {
          // Adjust steering angle based on the direction of the turn.
          if (race.direction == Direction::ANTICLOCKWISE) {
            current.steering_angle = Constants::MAX_RIGHT;
          } else if (race.direction == Direction::CLOCKWISE) {
            current.steering_angle = Constants::MAX_LEFT;
          }
          current.speed = -Constants::REDUCED_SPEED;  // Move the motor in reverse at reduced speed.
        }

        updateSteeringAngle();
      }
      break;
    case TurnState::CORRECTING:
      {
        // Fine-tuning the turn angle for precise positioning.
        race.collision_avoidance_blocked = false;  // Unblock collision avoidance.

        if (abs(angle_difference) <= 10) {
          turn_state = TurnState::COMPLETE;  // Transition to COMPLETE state.
        } else if (current.colour == Colour::RED || current.colour == Colour::GREEN) {
          race.collision_avoidance_blocked = false;
          obstacleSteering(current.x_pos, current.y_pos, current.colour);
        } else {
          angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
          // Adjust steering angle for fine-tuning the turn.
          if (angle_difference >= -10 && angle_difference <= 10) current.steering_angle = 90;
          if (angle_difference < -10 && angle_difference >= -20) current.steering_angle = 80;
          if (angle_difference > 10 && angle_difference <= 20) current.steering_angle = 105;
          if (angle_difference < -20) current.steering_angle = Constants::MAX_LEFT;
          if (angle_difference > 20) current.steering_angle = Constants::MAX_RIGHT;
        }

        current.speed = Constants::REDUCED_SPEED;  // Move the motor forward at reduced speed.
        updateSteeringAngle();
      }
      break;
    case TurnState::COMPLETE:
      {
        // Finalizing the turn, resetting the steering angle, and preparing for the next turn.
        race.sections++;
        updated_setpoint_angle = false;    // Reset the setpoint angle update flag.
        turn_state = TurnState::INITIATE;  // Reset the state to INITIATE for the next turn.
      }
      break;
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Initialization and Shutdown
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Captures initial measurement values to establish a baseline.
 *
 * Engages the IMU and front sonar to record the starting orientation and distance. This function
 * sets the initial conditions that future measurements will be compared against, ensuring accurate
 * relative readings throughout the robot's operation.
 */
void initGyroscope() {
  static bool is_initialized;

  if (!is_initialized) {
    mpu.calcOffsets();
    mpu.update();
    initial.yaw_angle = mpu.getAngleZ();
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
void initSonars() {
  static bool is_initialized;

  if (!is_initialized && current.distance_front != 0 && current.distance_front != 400) {
    initial.distance_front = current.distance_front;
    is_initialized = true;
  }
}

/**
 * @brief Initializes the relay.
 */
void initRelay() {
  static bool is_initialized;

  if (!is_initialized) {
    digitalWrite(Pins::RELAY_PIN, HIGH);
    delay(100);
    digitalWrite(Pins::RELAY_PIN, LOW);
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
void stop() {
  current.steering_angle = Constants::STRAIGHT;
  updateSteeringAngle();
  motor.end();
  race.enabled = false;
}

/**
 * @brief Manages the parking process of the robot.
 *
 * This function controls the parking process of the robot through a state machine approach,
 * ensuring precise handling of turns and movements based on the specified turn direction.
 * It consists of four states: INITIATE, TURNING, CORRECTING, and COMPLETE, to handle the entire
 * parking process from start to finish.
 *
 * @param turn_direction The direction in which the robot should turn to park (LEFT or RIGHT).
 */
void park(TurnDirection turn_direction) {
  // Enum for handling different turning states.
  enum class TurnState : const uint8_t {
    INITIATE,    // Initial state where the turn is prepared based on track conditions.
    TURNING,     // State where the robot is actively turning.
    CORRECTING,  // State for fine-tuning the turn angle.
    COMPLETE     // State indicating the turn has been completed.
  };

  // Constants for various parameters used in the function.
  const uint8_t CORRECTING_ANGLE_DIFFERENCE = 60;
  const uint16_t TURNING_DURATION = 3500;
  const uint16_t REVERSE_DURATION = 4000;
  const uint16_t INITIATION_DURATION = 1200;

  // Static variables for maintaining the current state and other control parameters.
  static TurnState turn_state;
  static bool updated_setpoint_angle;
  static bool initiating_timer_locked;
  static bool turning_timer_locked;
  int16_t angle_difference;
  static unsigned long initiating_timer;
  static unsigned long turning_timer;

  // Boolean flag to check if the turning timer has passed the specified duration.
  bool turning_timer_passed = (turning_timer_locked && millis() - turning_timer > TURNING_DURATION) ? 1 : 0;

  switch (turn_state) {
    case TurnState::INITIATE:
      {
        // Preparing to initiate a turn based on detected track conditions and vehicle orientation.
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

        race.collision_avoidance_blocked = false;

        // Lock the initiating timer if not already locked.
        if (!initiating_timer_locked) {
          initiating_timer = millis();
          initiating_timer_locked = true;
        }

        // Transition to TURNING state after the initiating duration has passed.
        if (initiating_timer_locked && millis() - initiating_timer > INITIATION_DURATION) {
          // Update the setpoint yaw angle based on the turn direction.
          if (!updated_setpoint_angle) {
            if (turn_direction == TurnDirection::LEFT) race.setpoint_yaw_angle += 90;
            else if (turn_direction == TurnDirection::RIGHT) race.setpoint_yaw_angle -= 90;
            updated_setpoint_angle = true;
          }
          angle_difference = race.setpoint_yaw_angle - current.yaw_angle;

          turn_state = TurnState::TURNING;  // Transition to TURNING state.
        } else {
          // Maintain the current path if the initiating duration has not passed.
          maintainStraightPath(angle_difference);
          current.speed = Constants::REDUCED_SPEED;
        }

        updateSteeringAngle();
      }
      break;
    case TurnState::TURNING:
      {
        race.collision_avoidance_blocked = true;  // Block collision avoidance during turn.

        // Calculate the angle difference between the setpoint and current yaw angle.
        angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
        if (abs(angle_difference) <= CORRECTING_ANGLE_DIFFERENCE) {
          turn_state = TurnState::CORRECTING;  // Transition to CORRECTING state.
        } else {
          // Adjust steering angle based on the direction of the turn.
          if (turn_direction == TurnDirection::LEFT) {
            current.steering_angle = Constants::MAX_LEFT;
          } else if (turn_direction == TurnDirection::RIGHT) {
            current.steering_angle = Constants::MAX_RIGHT;
          }
          current.speed = Constants::REDUCED_SPEED;  // Move the motor in reverse at reduced speed.
        }

        updateSteeringAngle();
      }
      break;
    case TurnState::CORRECTING:
      {
        static unsigned long reversing_timer;
        static bool reversing_timer_locked;

        race.collision_avoidance_blocked = true;  // Unblock collision avoidance.

        // Transition to COMPLETE state after the reverse duration has passed.
        if (reversing_timer_locked && millis() - reversing_timer > REVERSE_DURATION) {
          reversing_timer_locked = false;
          turn_state = TurnState::COMPLETE;  // Transition to COMPLETE state.
        } else {
          // Lock the reversing timer if not already locked.
          if (!reversing_timer_locked) {
            reversing_timer = millis();
            reversing_timer_locked = true;
          }

          // Calculate the angle difference and adjust the steering angle accordingly.
          angle_difference = race.setpoint_yaw_angle - current.yaw_angle;
          if (angle_difference >= -10 && angle_difference <= 10) current.steering_angle = 90;
          if (angle_difference < -10 && angle_difference >= -20) current.steering_angle = 80;
          if (angle_difference > 10 && angle_difference <= 20) current.steering_angle = 105;
          if (angle_difference < -20) current.steering_angle = Constants::MAX_LEFT;
          if (angle_difference > 20) current.steering_angle = Constants::MAX_RIGHT;
        }

        current.speed = -Constants::REDUCED_SPEED;  // Move the motor forward at reduced speed.
        updateSteeringAngle();
      }
      break;
    case TurnState::COMPLETE:
      {
        // Finalizing the parking process.
        updated_setpoint_angle = false;
        stop();
      }
      break;
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Sensor and Actuator Management
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Periodically refreshes sensor data to provide an accurate environmental model.
 *
 * Gathers and updates readings from the gyroscope, ultrasonic sensors, and voltage sensor at
 * regular intervals. This continuous refreshment of data is essential for the robot to make
 * informed decisions based on the latest information about its surroundings and internal state.
 */
void updateSensors() {
  const uint8_t IMU_FREQUENCY = 100;
  const uint8_t CAMERA_FREQUENCY = 20;
  static unsigned long last_millis_A;
  static unsigned long last_millis_B;

  initGyroscope();

  // Update the data stream of the gyroscope and the voltmeter.
  if (millis() - last_millis_A > (1000 / IMU_FREQUENCY)) {
    last_millis_A = millis();
    current.yaw_angle = (gyro.readYawAngle() - initial.yaw_angle) * 1.007;
    // current.yaw_angle = gyro.readYawAngle() - initial.yaw_angle;

    current.voltage = map(analogRead(Pins::VOLTAGE_MEASUREMENT_PIN), 0, 1023, 0, 100);
  }

  // Update the retrieved data of the pixy camera.
  if (race.obstacles_included && millis() - last_millis_B > (1000 / CAMERA_FREQUENCY)) {
    last_millis_B = millis();

    pixy.ccc.getBlocks();
    current.x_pos = camera.readX(race.magenta_unlocked);
    current.y_pos = camera.readY(race.magenta_unlocked);
    current.colour = camera.readColour(race.magenta_unlocked);
    current.block_index = camera.readIndex(race.magenta_unlocked);
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
void updateSteeringAngle() {
  // Transfer the steering angle to the servo, if changed.
  if (current.steering_angle != last.steering_angle) {
    last.steering_angle = current.steering_angle;
    servo.write(current.steering_angle);
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Navigation Control
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Manages the vehicle's lateral navigation during straight segments.
 *
 * Aligns the vehicle parallel to the track walls by adjusting the steering angle. The function
 * takes into account the race direction to optimize the wall-following behavior. It is a key
 * component in maintaining a steady course and preventing drift during straight-line travel.
 */
void trackInnerWall(uint16_t setpoint_distance) {
  // Calculate and store the output of the control loop
  if (race.direction == Direction::ANTICLOCKWISE) {
    trackLeftWall(setpoint_distance);
  } else if (race.direction == Direction::CLOCKWISE) {
    trackRightWall(setpoint_distance);
  }
}

/**
 * @brief Manages the vehicle's lateral navigation during straight segments.
 *
 * Aligns the vehicle parallel to the track walls by adjusting the steering angle. The function
 * takes into account the race direction to optimize the wall-following behavior. It is a key
 * component in maintaining a steady course and preventing drift during straight-line travel.
 */
void trackOuterWall(uint16_t setpoint_distance) {
  // Calculate and store the output of the control loop
  if (race.direction == Direction::ANTICLOCKWISE) {
    trackRightWall(setpoint_distance);
  } else if (race.direction == Direction::CLOCKWISE) {
    trackLeftWall(setpoint_distance);
  }
}

/**
 * @brief Navigates the robot centrally along the straight path.
 *
 * Ensures the robot maintains a central trajectory by dynamically adjusting the steering angle
 * based on the distance from the walls. This method is integral to the robot's ability to
 * navigate straight corridors without veering off course.
 */
void maintainCentralPath() {
  // Calculate and store the output of the control loop
  trackLeftWall(current.distance_right);
}

/**
 * @brief Maintains the robot's orientation by stabilizing angular velocity.
 *
 * Actively monitors and adjusts the robot's steering to counteract any unwanted rotational
 * movement. This method is crucial for ensuring the robot's orientation remains consistent,
 * particularly during maneuvers that could cause it to deviate from its intended heading.
 */
void maintainStraightPath(int16_t angle_difference) {
  // Calculate and store the output of the control loop
  if (angle_difference >= -2 && angle_difference <= 2) current.steering_angle = 90;
  if (angle_difference < -2 && angle_difference >= -10) current.steering_angle = 105;
  if (angle_difference > 2 && angle_difference <= 10) current.steering_angle = 80;
  if (angle_difference < -10) current.steering_angle = 120;
  if (angle_difference > 10) current.steering_angle = 60;
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
void trackLeftWall(uint16_t setpoint_distance) {
  // Check if the vehicle is within the acceptable range of the setpoint distance
  if (current.distance_left >= setpoint_distance - 1 && current.distance_left <= setpoint_distance + 1) {
    // If within range, keep the steering straight
    current.steering_angle = Constants::STRAIGHT;
  } else if (current.distance_left > setpoint_distance + 1) {
    // If too far from the wall, steer left
    current.steering_angle = 75;
  } else if (current.distance_left < setpoint_distance - 1) {
    // If too close to the wall, steer right
    current.steering_angle = 110;
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
uint16_t trackRightWall(uint16_t setpoint_distance) {
  // Evaluate if the vehicle's current distance from the right wall is within the setpoint range
  if (current.distance_right >= setpoint_distance - 1 && current.distance_right <= setpoint_distance + 1) {
    // Maintain a straight steering angle if within the target range
    current.steering_angle = Constants::STRAIGHT;
  } else if (current.distance_right > setpoint_distance + 1) {
    // Steer right if the vehicle is too far from the right wall
    current.steering_angle = 115;
  } else if (current.distance_right < setpoint_distance - 1) {
    // Steer left if the vehicle is too close to the right wall
    current.steering_angle = 80;
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Collision and Obstacle Management
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Evaluates the risk of collision based on sensor data.
 *
 * This function assesses the distances measured by the front and side ultrasonic sensors to
 * determine if the vehicle is at risk of colliding with an object. It is used to trigger
 * evasive maneuvers if necessary.
 *
 * @return True if a collision is imminent, false otherwise.
 */
bool collisionRisk() {
  if (current.distance_left <= Constants::MIN_DISTANCE
      || current.distance_right <= Constants::MIN_DISTANCE)
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
void avoidCollision() {
  if (current.distance_left <= Constants::MIN_DISTANCE) {
    current.steering_angle = 115;
  }
  if (current.distance_right <= Constants::MIN_DISTANCE) {
    current.steering_angle = 60;
  }

  if (!current.speed) current.speed = Constants::REDUCED_SPEED;
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
void obstacleSteering(uint16_t x, uint8_t y, Colour colour) {
  // Transfer a lowered speed to the motor to allow for precise maneuvering.
  current.speed = Constants::REDUCED_SPEED;

  // Define the proportional gain constant.
  const float Kp = 0.40;  // Adjust this value based on your system's requirements.

  // Calculate the setpoint based on the colour of the obstacle.
  float setpoint = 0.0;
  if (colour == Colour::RED) {
    // setpoint = (y - 207) / -3.450;
    setpoint = 15;
  } else if (colour == Colour::GREEN) {
    // setpoint = (y + 207) / 3.450;
    setpoint = 280;
  } else {
    // For other colours, maintain a neutral steering angle.
    maintainStraightPath(race.setpoint_yaw_angle - current.yaw_angle);
    return;
  }

  // Calculate the error between the setpoint and the current position.
  int16_t error = x - setpoint;

  // Compute the control output using the P-Controller.
  int16_t control_output = Kp * error;

  // Apply the control output to adjust the steering angle.
  // Ensure the steering angle remains within the valid range.
  current.steering_angle = constrain(90 + control_output, Constants::MAX_LEFT, Constants::MAX_RIGHT);
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Race Progress Monitoring
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Retrieves the number of sections completed by the vehicle.
 *
 * This function calculates the number of sections the vehicle has passed through in the current
 * lap. It is useful for tracking the vehicle's progress through the race course.
 *
 * @return The number of sections completed.
 */
uint8_t getSections() {
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
uint8_t getLaps() {
  race.laps = abs(current.yaw_angle) / 356;
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
bool detectedGap() {
  switch (race.direction) {
    case Direction::ANTICLOCKWISE:
      {
        if (current.distance_left >= Constants::MAX_DISTANCE) {
          return true;
        } else
          return false;
      }
      break;
    case Direction::CLOCKWISE:
      {
        if (current.distance_right >= Constants::MAX_DISTANCE) {
          return true;
        } else
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
bool getDirection() {
  static bool direction_determined;

  // Make sure that the direction is only determined once.
  if (!direction_determined) {
    if (current.distance_left >= 180 && current.distance_left != 400 && current.distance_left > current.distance_right) {
      race.direction = Direction::ANTICLOCKWISE;
      direction_determined = true;
      return true;
    } else if (current.distance_right >= 180 && current.distance_right != 400 && current.distance_right > current.distance_left) {
      race.direction = Direction::CLOCKWISE;
      direction_determined = true;
      return true;
    }
  } else {
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
bool detectedParkingLot() {
  static bool detected_parking_lot;

  if (current.colour == Colour::MAGENTA)
    detected_parking_lot = true;

  return detected_parking_lot;
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Sensor Data Processing
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

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
bool peak(uint16_t input, uint16_t threshold, uint8_t consecutive_matches) {
  static uint8_t matches;

  if (input >= threshold) {
    if (matches < consecutive_matches)
      matches++;

    if (matches >= consecutive_matches)
      return true;
  } else {
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
void cycleSonars() {
  const uint8_t FREQUENCY = 30;
  static uint8_t state;
  static unsigned long last_millis;

  switch (state) {
    case 0:
      {
        if (!sonarLeft.isUpdating()) {
          sonarFront.startMeasurement();
          if (millis() - last_millis > (1000) / FREQUENCY) {
            last_millis = millis();
            state++;
          }
        }
      }
      break;
    case 1:
      {
        if (!sonarFront.isUpdating()) {
          sonarRight.startMeasurement();
          if (millis() - last_millis > (1000) / FREQUENCY) {
            last_millis = millis();
            state++;
          }
        }
      }
      break;
    case 2:
      {
        if (!sonarRight.isUpdating()) {
          sonarLeft.startMeasurement();
          if (millis() - last_millis > (1000) / FREQUENCY) {
            last_millis = millis();
            state = 0;
          }
        }
      }
      break;
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Display and User Interface
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Visualizes sensor data on the LCD display for real-time monitoring.
 *
 * Executes periodic updates to the LCD display, presenting the latest sensor readings in an
 * easily interpretable format. This function enhances the user's ability to monitor the robot's
 * status and environmental interactions.
 */
void showData() {
  const uint8_t FREQUENCY = 5;
  const uint8_t LAYOUT_ID = race.obstacles_included ? 2 : 1;
  static unsigned long last_millis;

  if (millis() - last_millis > (1000 / FREQUENCY)) {
    last_millis = millis();

    // Print the display preset
    display.preset(LAYOUT_ID);

    // Update the data on the display
    switch (LAYOUT_ID) {
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
          display.update(last.yaw_angle, current.yaw_angle, 1, 1, 4, false);
          display.update(last.x_pos, current.x_pos, 6, 1, 3, false);
          display.update(last.y_pos, current.y_pos, 11, 1, 3, false);
        }
        break;
    }
  }
}


///ââââââââââââââââââââââââââââââââââââââââââââââââââ
/// @subsection Demonstrations
///ââââââââââââââââââââââââââââââââââââââââââââââââââ

/**
 * @brief Demonstrates the obstacle steering mechanism of the robot.
 * 
 * Executes the obstacle steering algorithm using the current position and color detected by the robot.
 * This function invokes the obstacleSteering method with the robot's current x and y positions and detected color,
 * then updates the steering angle accordingly. It serves as a demonstration or test of the obstacle steering functionality.
 */
void obstacleSteeringDemo() {
  obstacleSteering(current.x_pos, current.y_pos, current.colour);
  updateSteeringAngle();
}

/**
 * @brief Demonstrates the parking functionality of the robot.
 *
 * This function controls the parking process by first detecting a parking lot,
 * determining the direction to turn for parking, and then executing the parking maneuver.
 * It also handles collision avoidance and normal turning based on the current conditions.
 */
void parkingDemo() {
  // Static variables to maintain state across function calls.
  static TurnDirection turn_direction;       // Direction to turn for parking.
  static bool parking_lot_detected;          // Flag to indicate if a parking lot is detected.
  static bool parking_direction_determined;  // Flag to indicate if parking direction is determined.
  static uint8_t index;                      // Index of the detected parking lot block.

  // LAYER 1: Check if the robot is ready to initiate parking.
  if (race.sections >= 4) {
    race.magenta_unlocked = true;

    if (!race.turning && current.colour == Colour::MAGENTA || parking_lot_detected) {
      // Mark parking lot as detected.
      parking_lot_detected = true;

      // Determine the parking direction if not already determined.
      if (!parking_direction_determined) {
        if (current.x_pos < 157) turn_direction = TurnDirection::RIGHT;
        else if (current.x_pos >= 157) turn_direction = TurnDirection::LEFT;
        index = pixy.ccc.blocks[0].m_index;   // Store the index of the detected block.
        parking_direction_determined = true;  // Mark parking direction as determined.
      }

      // LAYER 2: Handle parking or other tasks based on current conditions.
      if (current.colour != Colour::MAGENTA || (current.colour == Colour::MAGENTA && index != pixy.ccc.blocks[0].m_index)) {
        park(turn_direction);
      } else if (collisionRisk() && !race.collision_avoidance_blocked) {
        avoidCollision();
      } else {
        // LAYER 3: Handle normal turning if no parking or collision risk.
        race.obstacles_included ? handleTurns(TurnMode::SWIFT) : handleTurns(TurnMode::SHARP);
      }
    }
  } else {
    // LAYER 2: Handle parking or other tasks based on current conditions.
    if (collisionRisk() && !race.collision_avoidance_blocked) {
      avoidCollision();
    } else {
      // LAYER 3: Handle normal turning if no parking or collision risk.
      race.obstacles_included ? handleTurns(TurnMode::SWIFT) : handleTurns(TurnMode::SHARP);
    }
  }
}