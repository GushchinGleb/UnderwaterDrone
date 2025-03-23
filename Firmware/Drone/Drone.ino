#include "motor.h"
#include "mpu6050.h"

#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();

  motor_init();
  mpu6050_init();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

long counter = 0;
short act = 0;

void loop() {
  int16_t mpu_data[7];
  mpu6050_getData(mpu_data);

  for (uint8_t i = 0; i < 7; ++i) {
    Serial.print(mpu_data[i]);
    Serial.print(' ');
  }
  Serial.print("                        \r");

  delay(500);
}
