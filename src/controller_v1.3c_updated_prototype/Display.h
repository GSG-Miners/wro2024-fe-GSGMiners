/**
 * @file Display.h
 * @brief Provides a set of commands for controlling a LiquidCrystal_I2C display.
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

struct Display {
  /**
   * @brief Formats a number into a string with a fixed number of digits and optional sign.
   * @param num The number to format.
   * @param max_digits The maximum number of digits to display.
   * @param show_sign Flag to determine if the sign should be displayed.
   * @return A formatted string representing the number.
   */
  String format(int16_t num, uint8_t max_digits, bool show_sign) {
    // Get the absolute value of the maximum and minimum number that
    // can be displayed according to max_digits
    uint16_t limit = pow(10, max_digits) - 1;

    // Convert number into string and get the sign from the number
    String str = String(abs(constrain(num, -limit, limit)));
    String sign = (num > 0) ? "+" : (num < 0) ? "-"
                                              : " ";

    // Add leading spaces until the string length reaches max_digits
    if (show_sign) {
      str = sign + str;
    }
    while (str.length() < max_digits + (show_sign ? 1 : 0)) {
      str = " " + str;
    }

    return str;
  }

  /**
   * @brief Updates the display with a new number at the specified location.
   * @param last_num The last number displayed.
   * @param current_num The current number to display.
   * @param cursor_x The x-coordinate of the cursor.
   * @param cursor_y The y-coordinate of the cursor.
   * @param max_digits The maximum number of digits to display.
   * @param show_sign Flag to determine if the sign should be displayed.
   */
  void update(int16_t last_num, int16_t current_num, uint8_t cursor_x, uint8_t cursor_y, uint8_t max_digits, bool show_sign) {
    if (last_num != current_num) {
      last_num = current_num;
      lcd.setCursor(cursor_x, cursor_y);
      lcd.print(format(current_num, max_digits, show_sign));
    }
  }

  /**
   * @brief Clears the display.
   */
  void clear() {
    for (uint8_t i = 0; i < ROWS; i++) {
      for (uint8_t j = 0; j < COLUMNS; j++) {
        lcd.setCursor(j, i);
        lcd.print(" ");
      }
    }
  }

  /**
   * @brief Displays a bootup message with a progress bar.
   */
  void bootup() {
    lcd.setCursor(2, 0);
    lcd.print("INITIALIZING");

    // Progress bar animation
    lcd.setCursor(2, 1);
    lcd.print("[----------]");
    delay(100);
    for (uint8_t i = 3; i < 13; i++) {
      lcd.setCursor(i, 1);
      lcd.print("=");
      delay(100);
    }
  }

  /**
   * @brief Displays preset labels on the screen.
   */
  void preset() {
    static bool preset_printed;

    if (!preset_printed) {
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
      preset_printed = true;
    }
  }

  /**
   * @brief Displays a shutdown message and initiates a countdown for power saving mode.
   */
  void shutdown() {
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
    for (uint8_t i = 2; i > 0; i--) {
      delay(500);
      lcd.setCursor(11, 1);
      lcd.print(i);
    }
    delay(500);
  }
};

#endif