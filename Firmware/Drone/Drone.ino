#include "gy-neo6mv2.h" // GPS sensor
#include "hmc5883l.h"   // magnitometer (compas)
#include "motor.h"      // drivers for the motors
#include "mpu6050.h"    // accelerometr
#include "RF24-rec.h"

#include "fast-math.h"

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

#define DBG_SER (Serial) // debug serial instance
#define DBG_SPEED (9600) // debug serial speed

#define CE_PIN  42  // CE pin for nRF24L01+
#define CSN_PIN 40  // CSN pin for nRF24L01+

static uint32_t t; // time from the start in milliseconds
static uint32_t e_l; // event: lost signal

static gy_neo6mv2_data_t gy_data; // gps data

RF24 radio(CE_PIN, CSN_PIN);

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
  RF24_init(radio, DBG_SER);

  t = millis();
  e_l = t + 2000; // add two seconds

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static uint8_t signal_lost = 0;
  static uint8_t motor_status = 0;

  t = millis();

  RF24_com_t RF24_com;

  if (RF24_command(radio, RF24_com)) { // command has been received
    if (RF24_com.chanel == CHS::O && RF24_com.value) { // valid online signal has arrived
      e_l = t + 2000; // schedule lost signal event to 2 seconds into the future
      if (signal_lost) {
        motor_chanel(CHS::l, 0);
        motor_chanel(CHS::r, 0);
        signal_lost = 0;
      }
    } else if (RF24_com.chanel) { // valid chanel
      motor_chanel(RF24_com.chanel, RF24_com.value);
    }
  }

  if (t > e_l) { // signal lost
    signal_lost = 1;
    if (t > e_l + 5000) { // 5 seconds left, ballast is empty
      motor_chanel(CHS::l, 0);
      motor_chanel(CHS::r, 0);
    } else {
      Serial.print("OFFLINE\r");
      // Stop and go up.
      motor_chanel(CHS::L, 0);
      motor_chanel(CHS::R, 0);
      motor_chanel(CHS::l, 140);
      motor_chanel(CHS::r, 140);
    }
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
