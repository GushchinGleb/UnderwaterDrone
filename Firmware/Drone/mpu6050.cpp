#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define RECALIBRATE

#define MPU_INT_PIN   48

static uint8_t fifoBuffer[45];

static void mean_sensors(MPU6050& mpu);

static const int buffersize = 70;     // количество итераций калибровки
static const int acel_deadzone = 8;  // точность калибровки акселерометра (по умолчанию 8)
static const int gyro_deadzone = 6;   // точность калибровки гироскопа (по умолчанию 2)
static int16_t ax, ay, az, gx, gy, gz;
static int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
static int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void mpu6050_calibrate(MPU6050& mpu) {
  mean_sensors(mpu);

  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    mean_sensors(mpu);
    Serial.print('.');
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= gyro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);
    if (abs(mean_gy) <= gyro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);
    if (abs(mean_gz) <= gyro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
    if (ready == 6) break;
  }
}

static void mean_sensors(MPU6050& mpu) {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) { // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2);
  }
}

void mpu6050_init(MPU6050& mpu) {
  Wire.begin();

  pinMode(MPU_INT_PIN, INPUT);

  // Enable pin change interrupt for PL1 (PCINT17)
  PCMSK1 |= (1 << PCINT17);  // Enable mask for PCINT17
  PCICR |= (1 << PCIE1);     // Enable PCINT group 1 (PCINT[14:21])

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

uint8_t mpu6050_event(MPU6050& mpu, int32_t positioning[9], float angles[3]) {
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return 0;
  }

  int32_t q[4]; // quaternion Q 30
  int16_t gravity[4];

  int32_t accel[3];

  mpu.dmpGetQuaternion(q, fifoBuffer);
  mpu.dmpGetGravity(gravity, fifoBuffer);
  // mpu.dmpGetYawPitchRoll(angles, &q, &gravity);

  mpu.dmpGetAccel(&positioning[0], fifoBuffer);
  // mpu.dmpGetLinearAccel(&aaReal, &accel, &gravity);
  // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  positioning[0] -= (int32_t)gravity[0] << 16;
  positioning[1] -= (int32_t)gravity[1] << 16;
  positioning[2] -= (int32_t)gravity[2] << 16;

  positioning[3] += positioning[0] >> 10; // Q 20
  positioning[4] += positioning[1] >> 10; // Q 20
  positioning[5] += positioning[2] >> 10; // Q 20
  positioning[6] += positioning[3] >> 10; // Q 10
  positioning[7] += positioning[4] >> 10; // Q 10
  positioning[8] += positioning[5] >> 10; // Q 10

  return 1;
}
