#include "MH-SD.h"

#include <Arduino.h>
#include <SD.h>

#define chipSelect 12

void MHSD_init() {

  Serial.println("Initializing SD card...");

  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  Serial.println("SD card is present and ready.");

  // Optionally, try to open a file to be sure
  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    char buff[10];
    buff[9] = '\0';
    testFile.readBytesUntil('\n', buff, 9);
    Serial.println(buff);
    Serial.println("Successfully opened test file for writing.");
    testFile.println("SD card write test OK.");
    testFile.write("test\n\n");
    testFile.close();
  } else {
    Serial.println("Failed to open file on SD card.");
  }
}