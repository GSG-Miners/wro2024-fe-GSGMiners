/**
 * @file Camera.h
 * @brief Header file for the Camera structure, interfacing with the Pixy2 camera module.
 *
 * The Camera structure encapsulates the functionality required to interact with the Pixy2 camera.
 * It provides methods to initialize the camera, read color signatures, and determine the position
 * of detected objects. The structure is designed to simplify the process of integrating the Pixy2
 * camera into robotics and automation projects, offering a high-level interface for object detection
 * and color recognition.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <sys/_stdint.h>
#include "includes/ra/fsp/src/bsp/mcu/all/bsp_arm_exceptions.h"
#include "api/Common.h"

#if defined(ARDUINO_ARCH_RENESAS)
#include <Pixy2_Renesas.h>
#else
#include <Pixy2.h>
#endif

/**
 * @enum Colour
 * @brief Enumerates the color signatures detected by the Pixy2 camera.
 * 
 * Provides symbolic names for the different colors that can be detected by the Pixy2 camera.
 * This enumeration simplifies the process of identifying and working with various color signatures
 * in the codebase, making it easier to handle color-based logic and decision-making.
 */
enum class Colour : const uint8_t {
  NONE,
  RED,
  GREEN,
  MAGENTA,
  WALL
};

// Create an instance of the Pixy2 camera.
Pixy2 pixy;

// Define signature constants for color detection.

#define RED_SIG_1 1
#define RED_SIG_2 3
#define RED_SIG_3 5
#define GREEN_SIG_1 2
#define GREEN_SIG_2 4
#define GREEN_SIG_3 6
#define MAGENTA_SIG 7

struct Camera {
  /**
   * @brief Initializes the Pixy2 camera and configures its lighting settings.
   *
   * Sets up the Pixy2 camera for operation by initializing its connection and configuring
   * the built-in lamp and LED to the desired state. The lamp is turned off, and the LED is
   * set to green, indicating readiness for color detection tasks.
   */
  void begin() {
    pixy.init();
    pixy.setLED(0, 0, 0);
  }

  /**
   * @brief Identifies the most prominent color detected by the camera.
   *
   * Analyzes the color signatures of detected objects and determines the most prominent
   * color based on predefined signatures. This method is useful for applications that need
   * to sort or identify objects by color.
   *
   * @return A value of the Colour enum representing the detected color, or Colour::NONE
   * if no color blocks are detected.
   */
  Colour readColour(bool magenta_unlocked) {
    if (pixy.ccc.numBlocks) {
      if (pixy.ccc.blocks[0].m_signature == RED_SIG_1 || pixy.ccc.blocks[0].m_signature == RED_SIG_2 || pixy.ccc.blocks[0].m_signature == RED_SIG_3) {
        return Colour::RED;
      } else if (pixy.ccc.blocks[0].m_signature == GREEN_SIG_1 || pixy.ccc.blocks[0].m_signature == GREEN_SIG_2 || pixy.ccc.blocks[0].m_signature == GREEN_SIG_3) {
        return Colour::GREEN;
      } else if (magenta_unlocked && pixy.ccc.blocks[0].m_signature == MAGENTA_SIG) {
        return Colour::MAGENTA;
      }
    } else {
      return Colour::NONE;
    }
  }

  /**
   * @brief Retrieves the horizontal position (X-coordinate) of the detected object.
   *
   * Captures the X-coordinate of the first detected object's position as seen by the camera.
   * This information can be used to determine the object's location within the camera's field
   * of view or to guide movement towards or away from the object.
   *
   * @return The X-coordinate of the first detected object, or 0 if no objects are detected.
   */
  uint16_t readX(bool magenta_unlocked) {
    if (pixy.ccc.numBlocks && magenta_unlocked && readColour(magenta_unlocked) == Colour::MAGENTA)
      return pixy.ccc.blocks[0].m_x;
    else if (pixy.ccc.numBlocks && !magenta_unlocked)
      return pixy.ccc.blocks[0].m_x;
    else
      return 0;
  }

  /**
   * @brief Retrieves the vertical position (Y-coordinate) of the detected object.
   *
   * Captures the Y-coordinate of the first detected object's position as seen by the camera.
   * This information is crucial for applications that require knowledge of an object's
   * position along the vertical axis within the camera's field of view.
   *
   * @return The Y-coordinate of the first detected object, or 0 if no objects are detected.
   */
  uint8_t readY(bool magenta_unlocked) {
    if (pixy.ccc.numBlocks && magenta_unlocked && readColour(magenta_unlocked) == Colour::MAGENTA)
      return pixy.ccc.blocks[0].m_y;
    else if (pixy.ccc.numBlocks && !magenta_unlocked)
      return pixy.ccc.blocks[0].m_y;
    else
      return 0;
  }

  /**
   *Â @brief Retrieves the index of the block.
   */
  uint8_t readIndex(bool magenta_unlocked) {
    if (pixy.ccc.numBlocks && magenta_unlocked && readColour(magenta_unlocked) == Colour::MAGENTA)
      return pixy.ccc.blocks[0].m_index;
    else if (pixy.ccc.numBlocks && !magenta_unlocked)
      return pixy.ccc.blocks[0].m_index;
    else
      return 0;
  }
};

#endif  // CAMERA_H