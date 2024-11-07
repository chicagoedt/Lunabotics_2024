#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2); 

void setup() {
  lcd.init(); 
  lcd.backlight(); 
  Serial.begin(9600); 
}

void loop() {
  static unsigned long lastUpdate = 0;
  static String displayLines[2];
  static int displayState = 0; // 0 for ip/ssid, 1 for temperature

  
  if (Serial.available() > 0) {
    
    String packetType = Serial.readStringUntil('\n');
    String line1 = Serial.readStringUntil('\n');
    String line2 = Serial.readStringUntil('\n');

    
    if (packetType == "IP_SSID") {
      displayLines[0] = line1;
      displayLines[1] = line2;
    } else if (packetType == "TEMP") {
      displayLines[0] = line1;
      displayLines[1] = line2;
    }
  }

  
  if (millis() - lastUpdate > 5000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(displayLines[0]);
    lcd.setCursor(0, 1);
    lcd.print(displayLines[1]);

   
    displayState = (displayState + 1) % 2;

    
    if (displayState == 0) {
      Serial.println("REQUEST_IP_SSID");
    } else {
      Serial.println("REQUEST_TEMP");
    }

    lastUpdate = millis();
  }
}
