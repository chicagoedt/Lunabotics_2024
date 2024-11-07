#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight

  Serial.begin(9600); // Initialize serial communication at 9600 bps
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming strings until newline characters
    String line1 = Serial.readStringUntil('\n');
    String line2 = Serial.readStringUntil('\n');

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(line1);

    lcd.setCursor(0, 1);
    lcd.print(line2);

    delay(5000);
  }
}
