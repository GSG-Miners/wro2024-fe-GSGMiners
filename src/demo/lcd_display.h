/**
 * @file lcd_display.h
 * @brief Expanded commands and specific routines for the LCD display.
<<<<<<< HEAD
 * @date 23rd December 2023 - 31st January 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch, Finnian Belger & Logan Weigoldt
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H
=======
 * @date 23rd December 2023 - 3rd February 2024
 * @author Maximilian Kautzsch
 * @details Last modified by Maximilian Kautzsch, Finnian Belger & Logan Weigoldt
*/
>>>>>>> 7156fcf (Large Update)

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <sys/_stdint.h>
#include <LiquidCrystal_I2C.h>

/**
 * @enum DisplayDimensions
 * @brief Enum to hold the specific amount of columns and rows of the lcd display.
<<<<<<< HEAD
 */
enum DisplayDimensions : const uint8_t
{
  kColumns = 16,
  kRows = 2
};

// Create LCD object
LiquidCrystal_I2C lcd(0x27, DisplayDimensions::kColumns, DisplayDimensions::kRows);
=======
*/
enum DisplayDimensions : const uint8_t {
  COLUMNS = 16,
  ROWS = 2
};

// Create LCD object
LiquidCrystal_I2C lcd(0x27, DisplayDimensions::COLUMNS, DisplayDimensions::ROWS);
>>>>>>> 7156fcf (Large Update)

/**
 * @brief Function to format numbers into strings with leading spaces.
 * @param number The number to format.
 * @param max_digits The maximum number of digits in the formatted string.
 * @param show_sign Checks if the sign of the number should be displayed.
 * @return The formatted string.
 */
<<<<<<< HEAD
String formatNumber(int16_t number, uint8_t max_digits, bool show_sign)
{
=======
String formatNumber(int16_t number, uint8_t max_digits, bool show_sign) {
>>>>>>> 7156fcf (Large Update)
  // Get the absolute value of the maximum and minimum number that
  // can be displayed according to max_digits
  uint16_t limit = pow(10, max_digits) - 1;

  // Convert number into string and get the sign from the number
  String str = String(abs(constrain(number, -limit, limit)));
  String sign = (number > 0)   ? "+"
                : (number < 0) ? "-"
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
 * @brief Function to update the LCD display with the current sensor readings.
 * @param last_number The last number displayed on the LCD.
 * @param current_number The current number to display on the LCD.
 * @param cursor_x The x position of the cursor on the LCD.
 * @param cursor_y The y position of the cursor on the LCD.
 * @param max_digits The maximum number of digits to display on the LCD.
 * @param show_sign Checks if the sign of the number should be displayed on the LCD.
 */
<<<<<<< HEAD
void lcdUpdate(int16_t last_number, int16_t current_number, uint8_t cursor_x, uint8_t cursor_y, uint8_t max_digits, bool show_sign)
{
  if (last_number != current_number)
  {
=======
void lcdUpdate(int16_t last_number, int16_t current_number, uint8_t cursor_x, uint8_t cursor_y, uint8_t max_digits, bool show_sign) {
  if (last_number != current_number) {
>>>>>>> 7156fcf (Large Update)
    lcd.setCursor(cursor_x, cursor_y);
    lcd.print(formatNumber(current_number, max_digits, show_sign));
    last_number = current_number;
  }
}

/**
 * @brief Clear the content of the LCD display without using large delays.
 */
<<<<<<< HEAD
void lcdClear()
{
  for (uint8_t i = 0; i < DisplayDimensions::kRows; i++)
  {
    for (uint8_t j = 0; j < DisplayDimensions::kColumns; j++)
    {
=======
void lcdClear() {
  for (uint8_t i = 0; i < DisplayDimensions::ROWS; i++) {
    for (uint8_t j = 0; j < DisplayDimensions::COLUMNS; j++) {
>>>>>>> 7156fcf (Large Update)
      lcd.setCursor(j, i);
      lcd.print(" ");
    }
  }
}

/**
 * @brief Bootup routine for the LCD display.
 */
void lcdBootup()
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
 * @brief Setup routine for printing the values on the LCD display.
<<<<<<< HEAD
 */
void lcdPrintValueSetup()
{
=======
*/
void lcdPrintValueSetup() {
>>>>>>> 7156fcf (Large Update)
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

/**
 * @brief Shutdown routine for the LCD display.
 */
void lcdShutdown()
{
  lcdClear();
  lcd.setCursor(6, 0);
  lcd.print("RACE");
  lcd.setCursor(4, 1);
  lcd.print("FINISHED");
  delay(1000);
  lcdClear();

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

<<<<<<< HEAD
#endif // LCD_DISPLAY_H
=======
#endif  // LCD_DISPLAY_H
>>>>>>> 7156fcf (Large Update)
