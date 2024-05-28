#include <sys/_stdint.h>
/**
 * @file Display.h
 * @brief Header file for display control, interfacing with LiquidCrystal_I2C displays.
 *
 * This file contains the declaration of a class that provides a simplified and
 * extended set of commands for controlling LiquidCrystal_I2C displays. It abstracts
 * the lower-level I2C communication, offering an intuitive API for display operations.
 *
 * @author Maximilian Kautzsch
 * @copyright Copyright (c) 2024 Maximilian Kautzsch
 * Licensed under MIT License.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <inttypes.h>
#include <LiquidCrystal_I2C.h>

#define COLUMNS 16
#define ROWS 2

LiquidCrystal_I2C lcd(0x27, COLUMNS, ROWS);

struct Display
{
  /**
   * @brief Formats a number into a string with a fixed number of digits and optional sign.
   *
   * This function takes an integer number and formats it into a string representation
   * that fits within a specified width, adding an optional sign prefix. It ensures that
   * the number is constrained within the displayable range and pads the string with spaces
   * for alignment purposes. This is useful for creating uniform and aligned numerical displays.
   *
   * @param num The number to format.
   * @param max_digits The maximum number of digits to display, excluding the sign.
   * @param show_sign If true, includes a '+' or '-' sign before the number.
   * @return A string formatted to the specified width with the number and optional sign.
   */
  String format(int16_t num, uint8_t max_digits, bool show_sign)
  {
    // Get the absolute value of the maximum and minimum number that
    // can be displayed according to max_digits
    uint16_t limit = pow(10, max_digits) - 1;

    // Convert number into string and get the sign from the number
    String str = String(abs(constrain(num, -limit, limit)));
    String sign = (num > 0) ? "+" : (num < 0) ? "-"
                                              : " ";

    // Add leading spaces until the string length reaches max_digits
    if (show_sign)
    {
      str = sign + str;
    }
    while (str.length() < max_digits + (show_sign ? 1 : 0))
    {
      str = " " + str;
    }

    return str;
  }

  /**
   * @brief Updates the display with a new number at the specified location.
   *
   * Compares the last displayed number with the current number and updates the display
   * only if there has been a change. This minimizes unnecessary writes to the display,
   * which can be time-consuming. The function positions the cursor and prints the formatted
   * number, ensuring that the display shows the most up-to-date value.
   *
   * @param last_num The previously displayed number.
   * @param current_num The new number to display.
   * @param cursor_x The horizontal position on the display where the number will be shown.
   * @param cursor_y The vertical position on the display where the number will be shown.
   * @param max_digits The maximum number of digits the number should occupy.
   * @param show_sign Whether to show the sign ('+' or '-') of the number.
   */
  void update(int16_t last_num, int16_t current_num, uint8_t cursor_x, uint8_t cursor_y, uint8_t max_digits, bool show_sign)
  {
    if (last_num != current_num || current_num == 0)
    {
      last_num = current_num;
      lcd.setCursor(cursor_x, cursor_y);
      lcd.print(format(current_num, max_digits, show_sign));
    }
  }

  /**
   * @brief Clears the entire display.
   *
   * Iterates over every position on the display and writes a space character to it,
   * effectively clearing the screen. This method is typically used before writing new
   * content to ensure the display is blank.
   */
  void clear()
  {
    for (uint8_t i = 0; i < ROWS; i++)
    {
      for (uint8_t j = 0; j < COLUMNS; j++)
      {
        lcd.setCursor(j, i);
        lcd.print(" ");
      }
    }
  }

  /**
   * @brief Displays a bootup message with a progress bar animation.
   *
   * Writes an "INITIALIZING" message on the display and creates a visual progress bar
   * that fills up over time. This provides a visual indication that the system is starting
   * up and can be used to enhance the user experience during the bootup sequence.
   */
  void bootup()
  {
    lcd.setCursor(2, 0);
    lcd.print("INITIALIZING");

    // Progress bar animation
    lcd.setCursor(2, 1);
    lcd.print("[----------]");
    delay(100);
    for (uint8_t i = 3; i < 13; i++)
    {
      lcd.setCursor(i, 1);
      lcd.print("=");
      delay(100);
    }
  }

  /**
   * @brief Displays preset labels on the screen for a consistent user interface.
   *
   * Prints a set of predefined labels at specific locations on the display. These labels
   * serve as static elements of the user interface, providing context for dynamic data
   * that will be displayed. The labels are printed only once to avoid unnecessary updates.
   *
   * @param layout_id The configuration identifier that determines the label layout.
   */
  void preset(uint8_t layout_id)
  {
    static bool preset_printed;

    if (!preset_printed)
    {
      // Clear the display to prepare for label printing.
      clear();

      // Handle different configuration numbers and their respective label layouts.
      switch (layout_id)
      {
      case 0:
      {
        lcd.setCursor(0, 0);
        lcd.print("L");
        lcd.setCursor(4, 0);
        lcd.print("M");
        lcd.setCursor(8, 0);
        lcd.print("R");
        lcd.setCursor(0, 1);
        lcd.print("X");
        lcd.setCursor(5, 1);
        lcd.print("Y");
        lcd.setCursor(10, 1);
        lcd.print("C");
        lcd.setCursor(13, 1);
        lcd.print("V");
      }
      break;
      case 1:
      {
        lcd.setCursor(0, 0);
        lcd.print("L");
        lcd.setCursor(5, 0);
        lcd.print("M");
        lcd.setCursor(10, 0);
        lcd.print("R");
        lcd.setCursor(0, 1);
        lcd.print("G");
        lcd.setCursor(10, 1);
        lcd.print("V");
      }
      break;
      case 2:
      {
        lcd.setCursor(0, 0);
        lcd.print("L");
        lcd.setCursor(5, 0);
        lcd.print("M");
        lcd.setCursor(10, 0);
        lcd.print("R");
        lcd.setCursor(0, 1);
        lcd.print("G");
        lcd.setCursor(5, 1);
        lcd.print("X");
        lcd.setCursor(10, 1);
        lcd.print("Y");
      }
      break;
      }

      // Mark the preset as printed to prevent future executions.
      preset_printed = true;
    }
  }

  /**
   * @brief Displays a shutdown message and initiates a countdown for power saving mode.
   *
   * Clears the display and shows a "RACE FINISHED" message followed by a countdown
   * indicating the transition to power saving mode. This function can be used to signal
   * the end of an operation and prepare the user for the device's power-down sequence.
   */
  void shutdown()
  {
    clear();
    lcd.setCursor(6, 0);
    lcd.print("RACE");
    lcd.setCursor(4, 1);
    lcd.print("FINISHED");
    delay(1000);
    clear();

    // Countdown for power saving mode
    lcd.setCursor(3, 0);
    lcd.print("POWER SAVING");
    lcd.setCursor(3, 1);
    lcd.print("MODE IN 3.");
    for (uint8_t i = 2; i > 0; i--)
    {
      delay(500);
      lcd.setCursor(11, 1);
      lcd.print(i);
    }
    delay(500);
  }
};

#endif // DISPLAY_H