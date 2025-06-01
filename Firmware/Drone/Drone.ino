#include "ballast.h"    // liquid ballast system
#include "gy-neo6mv2.h" // GPS sensor
#include "hmc5883l.h"   // magnitometer (compas)
#include "motor.h"      // drivers for the motors
#include "mpu6050.h"    // accelerometr
#include "RF24-rec.h"
#include "MH-SD.h"

// #include "fast-math.h"

#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

#define DBG_SER (Serial) // debug serial instance
#define DBG_SPEED (9600) // debug serial speed

#define CE_PIN  10  // CE pin for nRF24L01+  | 23 | PB4
#define CSN_PIN 11  // CSN pin for nRF24L01+ | 24 | PB5

static uint32_t t; // time from the start in milliseconds
static uint32_t e_l; // event: lost signal
static uint32_t e_s; // edge statuses
static uint32_t e_a; // accel event
static uint32_t e_r; // radio event

static gy_neo6mv2_data_t gy_data; // gps data

static uint8_t mpu6050_trigger = 0;

RF24 radio(CE_PIN, CSN_PIN);

MPU6050 mpu;

ISR(PCINT1_vect) {
  DBG_SER.println(F("int"));

  // Check if pin 48 (PL1) changed from LOW to HIGH
  if (PINL & (1 << PL1)) {
    mpu6050_trigger = 1;
  }
}

void setup() {
  DBG_SER.begin(DBG_SPEED);
  Wire.begin();

  Serial.print(F("Drone firmware: " __DATE__ " : " __TIME__ "\n\r"));
  Serial.print(F("F CPU: "));
  Serial.println(F_CPU);

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);

  ballast_init();
  gy_neo6mv2_init(gy_data);
  hmc5883_init();
  MHSD_init();
  motor_init();
  mpu6050_init(mpu);
  RF24_init(radio, DBG_SER);

  int16_t offsets[6] = {0,0,0,0,0,0};
// #define RECALIBRATE
#ifndef RECALIBRATE
  MHSD_read_offsets(offsets);
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset (offsets[3]);
  mpu.setYGyroOffset (offsets[4]);
  mpu.setZGyroOffset (offsets[5]);
#else
  mpu6050_calibrate(mpu);
  offsets[0] = mpu.getXAccelOffset();
  offsets[1] = mpu.getYAccelOffset();
  offsets[2] = mpu.getZAccelOffset();
  offsets[3] = mpu.getXGyroOffset();
  offsets[4] = mpu.getYGyroOffset();
  offsets[5] = mpu.getZGyroOffset();
  MHSD_write_offsets(offsets);
#endif // RECALIBRATE

  motor_chanel(CHS::R, 128);
  delay(500);
  motor_chanel(CHS::R, -128);
  delay(500);
  motor_chanel(CHS::R, 0);
  motor_chanel(CHS::L, 128);
  delay(500);
  motor_chanel(CHS::L, -128);
  delay(500);
  motor_chanel(CHS::L, 0);

  motor_chanel(CHS::r, 128);
  delay(500);
  motor_chanel(CHS::r, -128);
  delay(500);
  motor_chanel(CHS::r, 0);
  motor_chanel(CHS::l, 128);
  delay(500);
  motor_chanel(CHS::l, -128);
  delay(500);
  motor_chanel(CHS::l, 0);

  // t = millis();
  // e_l = t + 2000; // add two seconds

  if (DBG_SER.available()) {
    DBG_SER.println(F("RX TX short circuit"));
  }

  t = millis();
  e_a = t;
  e_l = t;
  e_r = t;
  e_s = t;
}

void loop() {
  static uint32_t counter = 0;
  ++counter;

  static uint8_t signal_lost = 0;
  static uint8_t motor_status = 0;

  t = millis();

  RF24_com_t RF24_com;

  if (t > e_r) {
    e_r += 10;

    if (RF24_command(radio, RF24_com)) { // command has been received
      if (RF24_com.chanel == CHS::O && RF24_com.value) { // valid online signal has arrived
        e_l = t + 5000; // schedule lost signal event to 5 seconds into the future
        if (signal_lost) {
          motor_chanel(CHS::l, 0);
          motor_chanel(CHS::r, 0);
          signal_lost = 0;
        }
      } else if (RF24_com.chanel) { // valid chanel
        motor_chanel(RF24_com.chanel, RF24_com.value);
      }
    }
  }

  static int32_t positioning[9] = {
    0, 0, 0, // access: X Y Z,
    0, 0, 0, // velocities: X Y Z
    0, 0, 0, // positions: X Y Z
  };
  static float angels[3] {
    0.f, 0.f, 0.f  // angles: yaw pitch roll
  };
  static uint32_t reads = 0;
  if (mpu6050_trigger) {
    // mpu6050_trigger = 0;
    // mpu6050_event(mpu, positioning);
    // ++reads;
  }

  if (t > e_a) {
    uint8_t read = mpu6050_event(mpu, positioning, angels);

    reads += read;
    if (read) {
      e_a += 10;
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
      motor_chanel(CHS::l, 0);
      motor_chanel(CHS::r, 0);
    }
  }

  // while (GY_SERIAL.available()) {
  //   DBG_SER.write(GY_SERIAL.read());
  // }

  if (t > e_s) {
    e_s = t + 1000;
    // DBG_SER.print("Edges: ");
    // DBG_SER.println((1 << 7) | ballast_check_all(), BIN);

    // int16_t data[3];
    // hmc5883_getValues(data);
    // for (uint8_t i= 0; i < 3; ++i) {
    //   Serial.print(data[i]);
    //   Serial.print(" ");
    // }
    // Serial.print(counter);
    // Serial.println("");
    // counter = 0;

    DBG_SER.print(reads);                                                        DBG_SER.print(" " );
    DBG_SER.print(positioning[0] / ((float)(1L << 30) / 9.8067f / 2));           DBG_SER.print(" " );
    DBG_SER.print(positioning[1] / ((float)(1L << 30) / 9.8067f / 2));           DBG_SER.print(" " );
    DBG_SER.print(positioning[2] / ((float)(1L << 30) / 9.8067f / 2));           DBG_SER.print("  ");
    DBG_SER.print(positioning[3] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print(" " );
    DBG_SER.print(positioning[4] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print(" " );
    DBG_SER.print(positioning[5] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print("  ");
    DBG_SER.print(positioning[6] / ((float)(1L << 10) / 9.8067f / 2 * 10000.f)); DBG_SER.print(" " );
    DBG_SER.print(positioning[7] / ((float)(1L << 10) / 9.8067f / 2 * 10000.f)); DBG_SER.print(" " );
    DBG_SER.print(positioning[8] / ((float)(1L << 10) / 9.8067f / 2 * 10000.f)); DBG_SER.println("");

    reads = 0;
  }
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
