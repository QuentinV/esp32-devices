#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(115200);

  // ESP32-C3 default I2C pins: SDA=8, SCL=9
  Wire.begin(8, 9);

  if (!sensor.init()) {
    Serial.println("Failed to detect VL53L0X!");
    while (1);
  }

  sensor.setTimeout(500);
  sensor.startContinuous();
}

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout!");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  delay(100);
}
