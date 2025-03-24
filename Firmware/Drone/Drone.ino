#include "hmc5883l.h"
#include "motor.h"
#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

void printData(const int16_t mpu[7], const int16_t hmc[3]);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  hmc5883_init();
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

  int16_t hmc_data[3];
  hmc5883_getValues(hmc_data);

  printData(mpu_data, hmc_data);

  delay(950);
}

void printData(const int16_t mpu[7], const int16_t hmc[3]) {
  for (uint8_t i = 0; i < 7; ++i) {
    Serial.print(mpu[i]);
    Serial.print(' ');
  }
  Serial.println("");

  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(mpu[i]);
    Serial.print(' ');
  }

  Serial.println("");
}
