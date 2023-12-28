/**
 * @file demo_006_controller.ino
 * @brief This sketch is used to test out an initial PID controller and
 * try out different tuning methods.
 * @author Maximilian Kautzsch
 * @date Created on 23rd December 2023
 * @date Last modified on 25th December 2023 by Maximilian Kautzsch,
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

/**
 * @enum Pins
 * @brief Enum to hold the pin numbers for the various components.
 */
enum Pins : const pin_size_t {
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
enum SteeringAngles : const uint8_t {
  kMaxLeft = 47,
  kMaxRight = 124,
  kStraight = 90,
};

/**
 * @struct SensorReadings
 * @brief Struct to group all measured input values (from the sensors).
 */
typedef struct {
  uint8_t button_count;     ///< Current button count
  uint8_t voltage;          ///< Current voltage reading
  uint16_t distance_left;   ///< Current distance reading from the left sensor
  uint16_t distance_front;  ///< Current distance reading from the front sensor
  uint16_t distance_right;  ///< Current distance reading from the right sensor
  int16_t yaw_angle;        ///< Current yaw angle reading
} SensorReadings;

/**
 * @struct ControllerParameters
 * @brief Struct to store all attributes that are used to configure a PID controller.
*/
typedef struct {
  float proportional_gain;        ///< Gain of proportional term
  float integral_gain;            ///< Gain of integral term
  float derivative_gain;          ///< Gain of derivative term
  ControllerDirection direction;  ///< Direction of the controller (direct or reverse)
} ControllerParameters;

// Initialize struct instances
SensorReadings lastReadings;
SensorReadings currentReadings;
ControllerParameters paramsDriveStraightPID = { 3.00, 1.20, 0.50, ControllerDirection::kDirect };

// Initialize the sensor and button objects
Button button(Pins::kButtonPin);
UltrasonicSensor sonarLeft(Pins::kTriggerPinLeft, Pins::kEchoPinLeft, Mode::kManual);
UltrasonicSensor sonarFront(Pins::kTriggerPinFront, Pins::kEchoPinFront, Mode::kManual);
UltrasonicSensor sonarRight(Pins::kTriggerPinRight, Pins::kEchoPinRight, Mode::kManual);
MPU6050 imu(Wire);
Motor motor(Pins::kMotorForwardPin, Pins::kMotorBackwardPin);
Servo servo;
PIDController driveStraightPID(paramsDriveStraightPID.direction);

/**
 * @brief Setup function for the Arduino sketch.
 */
void setup() {
  Serial.begin(9600);
  Wire.begin();

  imu.begin();
  lcd.init();
  lcd.backlight();

  servo.attach(Pins::kServoPin);
  motor.init();

  driveStraightPID.setLimits(SteeringAngles::kMaxLeft, SteeringAngles::kMaxRight);
  driveStraightPID.tune(paramsDriveStraightPID.proportional_gain, paramsDriveStraightPID.integral_gain, paramsDriveStraightPID.derivative_gain);
  driveStraightPID.setpoint(90);

  lcd.setCursor(2, 0);
  lcd.print("PRESS BUTTON");
  lcd.setCursor(4, 1);
  lcd.print("TO START");
}

/**
 * @brief Main loop function for the Arduino sketch.
 */
void loop() {
  button.update();
  sonarLeft.startMeasurement();
  sonarLeft.update();
  sonarFront.startMeasurement();
  sonarFront.update();
  sonarRight.startMeasurement();
  sonarRight.update();
  imu.update();
  motor.update();

  currentReadings.button_count = button.getCount();
  currentReadings.voltage = map(analogRead(Pins::kVoltageMeasurementPin), 0, 1023, 0, 100);
  currentReadings.distance_left = sonarLeft.getDistance();
  currentReadings.distance_front = sonarFront.getDistance();
  currentReadings.distance_right = sonarRight.getDistance();
  currentReadings.yaw_angle = imu.getAngleZ();

  if (currentReadings.button_count == 1) {
    static bool bootup_phase;

    switch (bootup_phase) {
      case 0:
        {
          imu.calcGyroOffsets();
          lcdBootup();
          lcdClear();
          bootup_phase = !bootup_phase;
        }
        break;
      case 1:
        {
          lcdPrintValues();
          driveStraightControlLoop();
        }
        break;
    }
  } else if (currentReadings.button_count == 2) {
    lcdShutdown();
    lcd.noBacklight();
    servo.write(90);
    motor.stop();
  }
}

/**
 * @brief Control loop that makes the robot drive straight in the centre
 of the parcour.
*/
void driveStraightControlLoop() {
  int16_t horizontal_distance, input, output;

  motor.setSpeed(70);

  horizontal_distance = currentReadings.distance_left + currentReadings.distance_right;
  input = map(currentReadings.distance_left, 0, horizontal_distance, 0, 180);
  driveStraightPID.update(input);
  output = driveStraightPID.getOutput();
  servo.write(output);

  driveStraightPID.serialPlotGraph();
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