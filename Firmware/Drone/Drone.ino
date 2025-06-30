#include "ballast.h"    // liquid ballast system
#include "chanels.h"
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
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define DBG_SER (Serial) // debug serial instance
#define DBG_SPEED (9600) // debug serial speed
#define SD_STORE (SD)    // SD card object

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

static uint8_t control_mode = 1; // 1 - direct, 2 - SAS, 3 - auto
static float target_angle = 0.f; // drone tries to head this angle
static int16_t axis_chs[2] = {0, 0}; // X, Y

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

  MHSD_init();

  ballast_init();
  gy_neo6mv2_init(gy_data);
  hmc5883_init();
  motor_init();
  mpu6050_init(mpu);
  RF24_init(radio, DBG_SER);

  motor_chanel(M_CS::M_R, 128);
  delay(500);
  motor_chanel(M_CS::M_R, -128);
  delay(500);
  motor_chanel(M_CS::M_R, 0);
  motor_chanel(M_CS::M_L, 128);
  delay(500);
  motor_chanel(M_CS::M_L, -128);
  delay(500);
  motor_chanel(M_CS::M_L, 0);

  motor_chanel(M_CS::M_r, 128);
  delay(500);
  motor_chanel(M_CS::M_r, -128);
  delay(500);
  motor_chanel(M_CS::M_r, 0);
  motor_chanel(M_CS::M_l, 128);
  delay(500);
  motor_chanel(M_CS::M_l, -128);
  delay(500);
  motor_chanel(M_CS::M_l, 0);

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

  static int32_t positioning[9] = {
    0, 0, 0, // access: X Y Z,
    0, 0, 0, // velocities: X Y Z
    0, 0, 0, // positions: X Y Z
  };
  static int16_t directions[3] {
    0.f, 0.f, 0.f  // angles: yaw pitch roll
  };
  static uint32_t reads = 0;
  if (mpu6050_trigger) {
    // mpu6050_trigger = 0;
    // mpu6050_event(mpu, positioning);
    // ++reads;
  }

  if (t > e_a) {
    uint8_t read = mpu6050_event(mpu, positioning, directions);

    reads += read;
    if (read) {
      e_a += 10;
    }
  }
  RF24_com_t RF24_com;
  static float d_angle = 0.f;
  static float target_d_angle = 0.f;

  if (t > e_r) {
    e_r += 10;

    if (RF24_command(radio, RF24_com)) { // command has been received
      if (RF24_com.chanel == RF_CS::RF_O && RF24_com.value) { // valid online signal has arrived
        e_l = t + 5000; // schedule lost signal event to 5 seconds into the future
        if (signal_lost) {
          motor_chanel(M_CS::M_l, 0);
          motor_chanel(M_CS::M_r, 0);
          signal_lost = 0;
        }
      } else if (RF24_com.chanel == RF_CS::RF_m) {
        if (RF24_com.value < 3) {
          control_mode = RF24_com.value;
        }
      } else if (RF24_com.chanel) { // valid chanel
        if (control_mode == 2) {
          switch (RF24_com.chanel) {
          case RF_CS::RF_X: axis_chs[0] = RF24_com.value; break;
          case RF_CS::RF_Y: target_angle += RF24_com.value / (255.f * 10.f); break;
          case RF_CS::RF_l: motor_chanel(M_CS::M_l, RF24_com.value); break;
          case RF_CS::RF_r: motor_chanel(M_CS::M_r, RF24_com.value); break;
          }
        } else {
          switch (RF24_com.chanel) {
          case RF_CS::RF_X: axis_chs[0] = RF24_com.value; break;
          case RF_CS::RF_Y: axis_chs[1] = RF24_com.value; break;
          case RF_CS::RF_l: motor_chanel(M_CS::M_l, RF24_com.value); break;
          case RF_CS::RF_r: motor_chanel(M_CS::M_r, RF24_com.value); break;
          }
        }
      }
    }

    if (control_mode == 2) {
      if (target_angle > F_PI) {
        target_angle -= F_2PI;
      } else if (target_angle < -F_PI) {
        target_angle += F_2PI;
      }
      static float last_diff = 0.f;
      const float angle = atan2(directions[1], directions[0]);
      float diff_a = target_angle - angle;
      if (diff_a > F_PI) {
        diff_a -= F_2PI;
      } else if (diff_a < -F_PI) {
        diff_a += F_2PI;
      }

      d_angle = (diff_a - last_diff) * 100.f;
      target_d_angle = f_sqrt(2.f * 1.0f * diff_a);

      axis_chs[1] = (int16_t)((target_d_angle - d_angle) * (255.f / F_PI));

      if (axis_chs[1] > 200) axis_chs[1] = 200;
      else if (axis_chs[1] < -200) axis_chs[1] = -200;

      last_diff = diff_a;
    }
    motor_control(axis_chs[0], axis_chs[1]);
  }

  if (t > e_l) { // signal lost
    if (!signal_lost) {
      // Serial.print("OFFLINE\r");
    }
    signal_lost = 1;
    if (t > e_l + 5000) { // 5 seconds left, ballast is empty
      motor_chanel(M_CS::M_l, 0);
      motor_chanel(M_CS::M_r, 0);
    } else {
      // Stop and go up.
      motor_chanel(M_CS::M_L, 0);
      motor_chanel(M_CS::M_R, 0);
      motor_chanel(M_CS::M_l, 0);
      motor_chanel(M_CS::M_r, 0);
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

    static uint32_t sample_id = 0;

    if (sample_id == 15) {
      sample_id++;
      for (uint8_t p = 3; p < 9; ++p) {
        positioning[p] = 0;
      }
    }
    ++sample_id;

    // DBG_SER.print(sample_id - 15);                                                    DBG_SER.print(" " );
    // DBG_SER.print(positioning[0] / ((float)(1L << 20) / 9.8067f / 2));           DBG_SER.print(" " );
    // DBG_SER.print(positioning[1] / ((float)(1L << 20) / 9.8067f / 2));           DBG_SER.print(" " );
    // DBG_SER.print(positioning[2] / ((float)(1L << 20) / 9.8067f / 2));           DBG_SER.print("  ");
    // DBG_SER.print(positioning[3] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print(" " );
    // DBG_SER.print(positioning[4] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print(" " );
    // DBG_SER.print(positioning[5] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print("  ");
    // DBG_SER.print(positioning[6] / ((float)(1L << 10) / 9.8067f / 2 * 10000.f)); DBG_SER.print(" " );
    // DBG_SER.print(positioning[7] / ((float)(1L << 10) / 9.8067f / 2 * 10000.f)); DBG_SER.print(" " );
    // DBG_SER.print(positioning[8] / ((float)(1L << 20) / 9.8067f / 2 * 100.f));   DBG_SER.print("  ");

    // DBG_SER.print(directions[0] / ((float)(1 << 14))); DBG_SER.print(" " );
    // DBG_SER.print(directions[1] / ((float)(1 << 14))); DBG_SER.print(" " );
    // DBG_SER.print(directions[2] / ((float)(1 << 14))); DBG_SER.print("  ");

    // DBG_SER.print(positioning[0], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[1], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[2], HEX); DBG_SER.print("  ");
    // DBG_SER.print(positioning[3], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[4], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[5], HEX); DBG_SER.print("  ");
    // DBG_SER.print(positioning[6], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[7], HEX); DBG_SER.print(" " );
    // DBG_SER.print(positioning[8], HEX); DBG_SER.print("  ");

    // DBG_SER.print(axis_chs[1]); DBG_SER.print("  ");
    // DBG_SER.print(target_angle); DBG_SER.print(" " );
    // DBG_SER.print(d_angle); DBG_SER.print(" " );
    // DBG_SER.print(target_d_angle); DBG_SER.println(" " );

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
