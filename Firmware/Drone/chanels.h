#pragma once

enum RF_CS { // RF24 chanels of RF24_command
  RF_INVALID = 0x0,

  RF_X, // front
  RF_Y, // rotate
  RF_l, // left ballast
  RF_r, // right ballast

  RF_O, // ONLINE

  RF_m, // mode, 1 - DIRECT, 2 - SAS, 3 - AUTO

  RF_COUNT
};

enum M_CS { // motor chanels of RF24_command
  M_INVALID = 0x0,

  M_L, // left motor
  M_R, // right motor
  M_l, // left ballast
  M_r, // right ballast

  M_COUNT
};
