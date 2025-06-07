#include "mpu6050.h"

#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define RECALIBRATE

#define MPU_INT_PIN   48

#define CONF_F F("mpu.txt")

static uint8_t fifoBuffer[45];

static uint8_t readRowUInt8(File& file);

static int16_t readRowInt16(File& file);

static uint8_t readRowUInt8(File& file) {
  char s = '\0';
  uint8_t result = 0;
  while (file.available() && s != '\n') {
    s = file.read();
    if ('0' <= s && s <= '9') {
      result *= 10;
      result += s - '0';
    }
  }

  return result;
}

static int16_t readRowInt16(File& file) {
  char s = '\0';
  int16_t result = 0;
  uint8_t sign = 0;
  while (file.available() && s != '\n') {
    s = file.read();
    if (s == '-') {
      sign = 1;
    } else if ('0' <= s && s <= '9') {
      result *= 10;
      result += s - '0';
    }
  }

  return sign ? -result : result;
}

static void mean_sensors(MPU6050& mpu);

// extern float sqrt_res;

/**
 * @param dir[OUT] Q6
 */
static uint8_t get_direction(const int32_t q[4], int16_t dir[3]);

static const int buffersize = 70;     // количество итераций калибровки
static const int acel_deadzone = 8;  // точность калибровки акселерометра (по умолчанию 8)
static const int gyro_deadzone = 6;   // точность калибровки гироскопа (по умолчанию 2)
static int16_t ax, ay, az, gx, gy, gz;
static int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
static int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

static uint8_t get_direction(const int32_t q[4], int16_t dir[3]) {
  const int8_t w = q[0] >> 23; // Q7
  const int8_t x = q[1] >> 23; // Q7
  const int8_t y = q[2] >> 23; // Q7
  const int8_t z = q[3] >> 23; // Q7

  // Optional: normalize (important if quaternion might be unnormalized)
  // float r_norm = f_rsqrt(w*w + x*x + y*y + z*z);
  // if (r_norm == 0.0f) return -1;
  // w *= r_norm;
  // x *= r_norm;
  // y *= r_norm;
  // z *= r_norm;

  // sqrt_res = ((int32_t)w*w + (int32_t)x*x + (int32_t)y*y + (int32_t)z*z) / (float)(1 << 6);

  // Rotate (1, 0, 0) using simplified quaternion rotation
  // Q14
  dir[0] = (1 << 14) - ((y*y + z*z) << 1);
  dir[1] = (x*y + w*z) << 1;
  dir[2] = (x*z - w*y) << 1;

  return 0;
}

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

  SD.remove(CONF_F);
  /**
   * - X accel [-32768; 32767]
   * - Y accel [-32768; 32767]
   * - Z accel [-32768; 32767]
   * - X gyro  [-32768; 32767]
   * - Y gyro  [-32768; 32767]
   * - Z gyro  [-32768; 32767]
   */
  File MPU_options = SD.open(CONF_F, FILE_WRITE);
  if (!MPU_options) {
    return;
  }

  MPU_options.println(mpu.getXAccelOffset());
  MPU_options.println(mpu.getYAccelOffset());
  MPU_options.println(mpu.getZAccelOffset());
  MPU_options.println(mpu.getXGyroOffset());
  MPU_options.println(mpu.getYGyroOffset());
  MPU_options.println(mpu.getZGyroOffset());

  MPU_options.close();
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

  /**
   * - X accel [-32768; 32767]
   * - Y accel [-32768; 32767]
   * - Z accel [-32768; 32767]
   * - X gyro  [-32768; 32767]
   * - Y gyro  [-32768; 32767]
   * - Z gyro  [-32768; 32767]
   */
  File MPU_options = SD.open(CONF_F, FILE_READ);
  if (!MPU_options) {
    MPU_options.close();
    SD.remove(CONF_F);
    File MPU_options = SD.open(CONF_F, FILE_WRITE);
    if (!MPU_options) {
      MPU_options.close();
      Serial.println(F("Failed to open mpu.conf"));
      return;
    }

    const char data[] PROGMEM = "0\n0\n0\n0\n0\n0\n";
    MPU_options.write(data, sizeof(data) - 1);

    MPU_options.close();
    return;
  }

  mpu.setXAccelOffset(readRowUInt8(MPU_options));
  mpu.setYAccelOffset(readRowUInt8(MPU_options));
  mpu.setZAccelOffset(readRowUInt8(MPU_options));
  mpu.setXGyroOffset(readRowUInt8(MPU_options));
  mpu.setYGyroOffset(readRowUInt8(MPU_options));
  mpu.setZGyroOffset(readRowUInt8(MPU_options));

  MPU_options.close();

  Serial.println(F("MPU compete"));
}

uint8_t mpu6050_event(MPU6050& mpu, int32_t positioning[9], int16_t direction[3]) {
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return 0;
  }

  int32_t q[4]; // quaternion Q 30
  int16_t gravity[4];

  mpu.dmpGetQuaternion(q, fifoBuffer);
  mpu.dmpGetGravity(gravity, fifoBuffer);
  // mpu.dmpGetYawPitchRoll_i(angles, q, gravity);
  get_direction(q, direction);

  int16_t accel_relative[3];
  mpu.dmpGetAccel(accel_relative, fifoBuffer);

  const int16_t accel_real[3] = {
    accel_relative[0] - gravity[0],
    accel_relative[1] - gravity[1],
    accel_relative[2] - gravity[2]
  };

  // mpu.dmpGetLinearAccelInWorld_i(positioning, accel_real, q);

  // positioning[0] += (int32_t)( 0.012f  * ((float)(1L << 20) / 9.8067f / 2));
  // positioning[1] += (int32_t)( 0.0901f * ((float)(1L << 20) / 9.8067f / 2));
  // positioning[2] += (int32_t)(-0.008f  * ((float)(1L << 20) / 9.8067f / 2));

  // memcpy(positioning, accel_relative, sizeof(accel_relative));

  positioning[0] = accel_real[0];
  positioning[1] = accel_real[1];
  positioning[2] = accel_real[2];

  // positioning[3] = gravity[0];
  // positioning[4] = gravity[1];
  // positioning[5] = gravity[2];

  // positioning[6] = accel_relative[0];
  // positioning[7] = accel_relative[1];
  // positioning[8] = accel_relative[2];

  positioning[3] += positioning[0]; // Q 20
  positioning[4] += positioning[1]; // Q 20
  positioning[5] += positioning[2]; // Q 20
  positioning[6] += positioning[3] >> 10; // Q 10
  positioning[7] += positioning[4] >> 10; // Q 10
  positioning[8] += positioning[5] >> 10; // Q 10

  return 1;
}

void mpu6050_measure(MPU6050& mpu, int16_t m[6 * 3]) {
  for (uint8_t measure = 0; measure < 6; ++measure) {
    Serial.println(F("Orientate..."));
    delay(3000);
    Serial.println(F("Start"));
    mpu6050_calibrate(mpu);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.getAcceleration(&m[measure * 3 + 0], &m[measure * 3 + 1], &m[measure * 3 + 2]);
  }
  Serial.println(F("Measurement completed"));
}
