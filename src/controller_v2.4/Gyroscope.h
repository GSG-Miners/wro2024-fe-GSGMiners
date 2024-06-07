/**
 * @file Gyroscope.h
 * @brief Header file for the Gyroscope structure, interfacing with the MPU6050 sensor.
 *
 * The Gyroscope structure encapsulates the functionality required to interact with the MPU6050 sensor.
 * It provides methods to initialize the sensor and read the yaw angle. The structure is designed to simplify
 * the process of integrating the MPU6050 gyroscope into robotics and automation projects, offering a high-level
 * interface for orientation detection and stabilization tasks.
 */

#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <sys/_stdint.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Create an instance of the MPU6050 sensor.
MPU6050 mpu(Wire);

/**
 * @struct Gyroscope
 * @brief Structure to manage the MPU6050 gyroscope sensor.
 *
 * The Gyroscope structure provides methods to initialize the MPU6050 sensor and read its yaw angle.
 * It handles the setup and configuration of the sensor and offers a simple interface for obtaining
 * orientation data.
 */
struct Gyroscope {
  /**
   * @brief Initializes the MPU6050 sensor and configures its settings.
   *
   * Sets up the MPU6050 sensor for operation by initializing the I2C communication and configuring
   * the sensor's filter coefficient for gyroscopic measurements. This method ensures the sensor
   * is ready to provide accurate orientation data.
   */
  void begin() {
    Wire.begin();

    bool status = mpu.begin();
    while (status != 0) {}
    mpu.setFilterGyroCoef(1.00);
  }

  /**
   * @brief Reads the current yaw angle from the MPU6050 sensor.
   *
   * Updates the sensor data and retrieves the yaw angle, which represents the rotation around
   * the vertical axis. This method provides real-time orientation information that can be used
   * for navigation and stabilization purposes.
   *
   * @return The current yaw angle in degrees as an int16_t value.
   */
  int16_t readYawAngle() {
    mpu.update();
    return mpu.getAngleZ();
  }
};

#endif  // GYROSCOPE_H