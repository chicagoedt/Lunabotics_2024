#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2); 

void setup() {
  lcd.init(); 
  lcd.backlight(); 

  Serial.begin(9600); 
}

void loop() {
  if (Serial.available() > 0) {
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
