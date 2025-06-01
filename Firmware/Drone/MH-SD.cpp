#include "MH-SD.h"

#include <Arduino.h>
#include <SD.h>
#include <stdint.h>

#define chipSelect 12

void MHSD_init() {

  Serial.println(F("Initializing SD card..."));

  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD card initialization failed!"));
    return;
  }

  Serial.println(F("SD card is present and ready."));

  // Optionally, try to open a file to be sure
  File test_file = SD.open(F("test.txt"), FILE_WRITE);
  if (test_file) {
    Serial.println(F("Successfully opened test file for writing."));
    test_file.println(F("SD card write test OK."));
    test_file.close();
  } else {
    Serial.println(F("Failed to open file on SD card."));
  }
}

uint8_t MHSD_read_offsets(int16_t offsets[6]) {
  offsets[0] = 0;
  offsets[1] = 0;
  offsets[2] = 0;
  offsets[3] = 0;
  offsets[4] = 0;
  offsets[5] = 0;

  File offsets_file = SD.open(F("offsets.txt"));
  if (!offsets_file) {
    return 0;
  }

  uint8_t counter = 0;
  uint8_t sign = 0;
  while (offsets_file.available()) {
    char byte = '\0';
    offsets_file.read(&byte, 1);
    if (byte == '-') {
      sign = 1;
    } else if (byte >= '0' && byte <= '9') {
      offsets[counter] *= 10;
      offsets[counter] += byte - '0';
    } else if (byte == '\n') {
      if (sign) {
        offsets[counter] = -offsets[counter];
      }
      ++counter;
      sign = 0;
      if (counter >= 6) {
        break;
      }
    }
  }

  offsets_file.close();

  return counter;
}

void MHSD_write_offsets(const int16_t offsets[6]) {
  SD.remove(F("offsets.txt"));
  File offsets_file = SD.open(F("offsets.txt"), FILE_WRITE);
  if (!offsets_file) {
    return;
  }

  offsets_file.println(offsets[0]);
  offsets_file.println(offsets[1]);
  offsets_file.println(offsets[2]);
  offsets_file.println(offsets[3]);
  offsets_file.println(offsets[4]);
  offsets_file.println(offsets[5]);
  offsets_file.close();
}