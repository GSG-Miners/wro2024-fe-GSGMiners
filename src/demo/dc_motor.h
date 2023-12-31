/**
 * @file dc_motor.h
 * @brief DC motor control class using the "pwm.h" module from the arduino-renesas core.
 * @date 23rd December 2023 - 31st January 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch, Finnian Belger & Logan Weigoldt
 */
== == == =
             *@date 23rd December 2023 - 3rd February 2024 * @author Maximilian Kautzsch * @details Last modified by Maximilian Kautzsch,
      Finnian Belger &Logan Weigoldt
              * /
>>>>>>> 7156fcf (Large Update)

#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <sys/_stdint.h>
#include <Arduino.h>
#include "pwm.h"

          /**
           * @enum MotorConstants
           * @brief This enum type defines the motor constants for speed, acceleration and update interval.
          <<<<<<< HEAD
           */
          enum MotorConstants : const int8_t {
            == == == =
                         * /
                         enum MotorConstants : const int8_t{
>>>>>>> 7156fcf (Large Update)
                             MIN_SPEED = -100,
                             MAX_SPEED = 100,
                             MIN_ACCELERATION = 0,
                             MAX_ACCELERATION = 100,
                             MIN_UPDATE_INTERVAL = 50,
                             MAX_UPDATE_INTERVAL = 2};

            /**
             * @class Motor
             * @brief Class for controlling a DC motor.
             */
            class Motor{
              public :
                  Motor(uint8_t forward_pin, uint8_t backward_pin);
              ~Motor();

              bool init();
              void update();
              void setSpeed(int8_t speed);
              void setAcceleration(uint8_t acceleration);
              bool stop();
              bool isUpdating();
              int8_t getSpeed();

              private :
                  PwmOut forwardPwm;                 ///< The PWM output for the forward direction of the motor
              PwmOut backwardPwm;                    ///< The PWM output for the backward direction of the motor
              bool enabled_;                         ///< The flag to indicate if the motor is enabled or disabled
              bool status_;                          ///< The flag to indicate if the motor is updating its speed
              int8_t current_speed_;                 ///< The current speed of the motor in percentage, from -100 (backward) to 100 (forward)
              int8_t setpoint_speed_;                ///< The desired speed of the motor in percentage, from -100 (backward) to 100 (forward)
              uint8_t acceleration_;                 ///< The acceleration of the motor in percentage, from 0 (no acceleration) to 100 (maximum acceleration)
              uint8_t speed_update_interval_;        ///< The interval in milliseconds to update the motor speed
              uint16_t frequency_;                   ///< The frequency of the PWM signal in Hz
              unsigned long last_speed_update_time_; ///< The last time in milliseconds when the motor speed was updated
            };

#endif // DC_MOTOR_H