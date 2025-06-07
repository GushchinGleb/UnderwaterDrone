#pragma once

#include <Arduino.h>
#include <RF24.h>
#include <stdint.h>

enum CHS { // RF24 chanels of RF24_command
  INVALID = 0x0,

  X, // left motor
  Y, // right motor
  l, // left ballast
  r, // right ballast

  O, // ONLINE

  m, // mode 1 - direct, 2 - SAS, 3 - auto

  CHS_COUNT
};

typedef struct RF24_command {
  uint8_t chanel;
  int16_t value;
} RF24_com_t; // RF24 command

/**
 * The function initializes the RF24 module
 * @param radio[OUT] link to the module
 * @param dbg_ser[IN|OUT] debug output to the serial
 */
void RF24_init(RF24& radio, HardwareSerial& dbg_ser);

/**
 * The function sends a command from the RF24
 * @param radio[IN|OUT] RF24 object
 * @param command[OUT] command to the RF24
 */
void RF24_send(RF24& radio, RF24_com_t& command);
