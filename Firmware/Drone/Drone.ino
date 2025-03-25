#include "gy-neo6mv2.h" // GPS sensor
#include "hmc5883l.h"   // magnitometer (compas)
#include "motor.h"      // drivers for the motors
#include "mpu6050.h"    // accelerometr

#include "fast-math.h"

#include <Arduino.h>
#include <Wire.h>

#define DBG_SER (Serial) // debug serial instance
#define DBG_SPEED (9600) // debug serial speed

static uint32_t t; // time from the start in milliseconds
static uint32_t e_p; // event: print data

static gy_neo6mv2_data_t gy_data; // gps data

void printData(const int16_t mpu[7], const int16_t hmc[3]);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  DBG_SER.begin(DBG_SPEED);
  Wire.begin();

  gy_neo6mv2_init(gy_data);
  hmc5883_init();
  motor_init();
  mpu6050_init();

  t = millis();
  e_p = t;

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  t = millis();

  if (t > e_p) {
    e_p = t + 1000; // + 1 second

    digitalWrite(LED_BUILTIN, HIGH);

    int16_t mpu_data[7];
    mpu6050_getData(mpu_data);

    int16_t hmc_data[3];
    hmc5883_getValues(hmc_data);

    printData(mpu_data, hmc_data);

    float in_val = asin(0.43f);
    DBG_SER.println(in_val);
    float val = f_sin_f(in_val);
    DBG_SER.println(val);

    digitalWrite(LED_BUILTIN, LOW);
  }

  while (GY_SERIAL.available()) {
    DBG_SER.write(GY_SERIAL.read());
  }
}

void printData(const int16_t mpu[7], const int16_t hmc[3]) {
  for (uint8_t i = 0; i < 7; ++i) {
    DBG_SER.print(mpu[i]);
    DBG_SER.print(' ');
  }
  DBG_SER.println("");

  for (uint8_t i = 0; i < 3; ++i) {
    DBG_SER.print(mpu[i]);
    DBG_SER.print(' ');
  }

  DBG_SER.println("");
}

void printGY(const gy_neo6mv2_data_t& gy_data) {
  DBG_SER.print(F("Time: "));
  DBG_SER.print(gy_data.time[0]); DBG_SER.print(":");
  DBG_SER.print(gy_data.time[1]); DBG_SER.print(":");
  DBG_SER.println(gy_data.time[2]);

  DBG_SER.print(F("Latitude:  ")); DBG_SER.println(gy_data.lat, 6);
  DBG_SER.print(F("Longitude: ")); DBG_SER.println(gy_data.lon, 6);
  DBG_SER.print(F("Fix:       ")); DBG_SER.println(gy_data.fix);
  DBG_SER.print(F("Altitude:  ")); DBG_SER.print(gy_data.alt); DBG_SER.println(" m");
  DBG_SER.print(F("Sea Level: ")); DBG_SER.print(gy_data.sea); DBG_SER.println(" m");
}
