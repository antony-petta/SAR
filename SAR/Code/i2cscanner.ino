#include <Wire.h>

// Define custom I2C pins
#define CUSTOM_SDA_PIN 21
#define CUSTOM_SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting I2C scanner...");

  // Setup custom I2C pins
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);

  Serial.println("Scanning...");

  byte count = 0;

  for (byte i = 1; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at address 0x");
      if (i < 16) Serial.print("0");
      Serial.print(i, HEX);
      Serial.println(" !");
      count++;
    }
  }

  Serial.println("Scan completed.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" devices.");

  delay(5000); // You can adjust this delay as needed
}

void loop() {
  // Nothing to do here
}
