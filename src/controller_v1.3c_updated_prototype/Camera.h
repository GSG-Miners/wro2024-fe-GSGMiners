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

#if defined(ARDUINO_ARCH_RENESAS)
#include <Pixy2_Renesas.h>
#else
#include <Pixy2.h>
#endif

// Create an instance of the Pixy2 camera.
Pixy2 pixy;

// Define constants for location updates.
#define LOCATION_A 1
#define LOCATION_B 2
#define LOCATION_C 3

// Define signature constants for color detection.
#define RED_SIG_1 1
#define RED_SIG_2 3
#define RED_SIG_3 5
#define GREEN_SIG_1 2
#define GREEN_SIG_2 4
#define GREEN_SIG_3 6
#define MAGENTA_SIG 7

struct Camera
{
    uint8_t updating_location;

    /**
     * @brief Initializes the Pixy2 camera and configures its lighting settings.
     *
     * Sets up the Pixy2 camera for operation by initializing its connection and configuring
     * the built-in lamp and LED to the desired state. The lamp is turned off, and the LED is
     * set to green, indicating readiness for color detection tasks.
     */
    void begin()
    {
        pixy.init();
        pixy.setLamp(0, 1);     // upper and lower LEDs of the lamp
        pixy.setLED(0, 255, 0); // set RGB LED to green (R: 0, G: 255, B: 0)
    }

    /**
     * @brief Identifies the most prominent color detected by the camera.
     *
     * Analyzes the color signatures of detected objects and determines the most prominent
     * color based on predefined signatures. This method is useful for applications that need
     * to sort or identify objects by color.
     *
     * @return A character representing the detected color ('r' for red, 'g' for green) or
     * a space character if no color blocks are detected.
     */
    char readColour()
    {
        if (updating_location != LOCATION_B && updating_location != LOCATION_C)
        {
            updating_location = LOCATION_A;
            pixy.ccc.getBlocks();
        }

        if (pixy.ccc.numBlocks)
        {
            for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
            {
                // Calculate the area for red and green blocks.
                if (pixy.ccc.blocks[i].m_signature == RED_SIG_1 || pixy.ccc.blocks[i].m_signature == RED_SIG_2 || pixy.ccc.blocks[i].m_signature == RED_SIG_3)
                {
                    return 'r';
                }
                else if (pixy.ccc.blocks[i].m_signature == GREEN_SIG_1 || pixy.ccc.blocks[i].m_signature == GREEN_SIG_2 || pixy.ccc.blocks[i].m_signature == GREEN_SIG_3)
                {
                    return 'g';
                }
            }
        }
        else
            return ' ';
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
    uint16_t readX()
    {
        if (updating_location != LOCATION_A && updating_location != LOCATION_C)
        {
            updating_location = LOCATION_B;
            pixy.ccc.getBlocks();
        }

        if (pixy.ccc.numBlocks)
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
    uint8_t readY()
    {
        if (updating_location != LOCATION_A && updating_location != LOCATION_B)
        {
            updating_location = LOCATION_C;
            pixy.ccc.getBlocks();
        }

        if (pixy.ccc.numBlocks)
            return pixy.ccc.blocks[0].m_y;
        else
            return 0;
    }
};

#endif // CAMERA_H