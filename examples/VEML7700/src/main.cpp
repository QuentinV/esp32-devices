#include <Wire.h>
#include <Adafruit_VEML7700.h>

Adafruit_VEML7700 veml;

void setup() {
  Serial.begin(115200);

  // ESP32-C3 default I2C pins: SDA=8, SCL=9
  Wire.begin(8, 9);

  if (!veml.begin()) {
    Serial.println("VEML7700 not found!");
    while (1);
  }

  Serial.println("VEML7700 initialized");

  // Optional tuning
  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_100MS);
}

void loop() {
  float lux = veml.readLux();
  Serial.print("Ambient light: ");
  Serial.print(lux);
  Serial.println(" lux");

  delay(1000);
}
