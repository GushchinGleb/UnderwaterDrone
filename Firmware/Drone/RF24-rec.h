#pragma once

#include <stdint.h>

#include <RF24.h>

typedef struct RF24_command {
  uint8_t chanel;
  int16_t value;
} RF24_com_t; // RF24 command

/**
 * @brief The function initializes RF24 radio
 * @param radio[IN|OUT] radio object
 * @param dbg_ser[IN|OUT] serial port for debug messages
 */
void RF24_init(RF24& radio, HardwareSerial& dbg_ser);

/**
 * @brief The function receive command from radio.
 * @param radio[IN] radio object
 * @param command[OUT] command from the radio
 * @return true if command have been received or false if not
 */
bool RF24_command(RF24& radio, RF24_com_t& command);
