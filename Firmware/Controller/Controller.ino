#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "RF24-snd.h"

#define CE_PIN 9
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);

bool send_from_serial();
void send_ONLINE();

uint32_t t; // current time millis

void setup() {
  Serial.begin(9600);

  RF24_init(radio, Serial);

  t = millis();
}

void loop() {
  static uint32_t send_tmg = t; // send ONLINE timing

  t = millis();

  if (send_from_serial()) {
    send_tmg = t + 500; // 0.5s delay
  }

  if (t > send_tmg) {
    send_tmg = t + 500;
    send_ONLINE();
  }
}

bool send_from_serial() {
  if (!Serial.available()) {
    return false;
  }

  String data = Serial.readStringUntil('\n'); // Read from Serial Monitor
  Serial.print("Sending: ");
  Serial.println(data.c_str());

  /**
   * @brief the command will send to the drone
   * - X -- X axis
   * - Y -- rotate
   * - Z -- depth
   * - O -- ONLINE
   */
  char command_c = '\0';
  RF24_com_t command;
  sscanf(data.c_str(), "%c %i", &command_c, &command.value);
  Serial.print("Char(HEX): ");
  Serial.print(command_c, HEX);
  Serial.print(" -> CH: ");

  switch (command_c) {
  case 'x': command.chanel = CHS::X; break;
  case 'y': command.chanel = CHS::Y; break;
  case 'z': command.chanel = CHS::Z; break;
  default: return false;
  }

  Serial.print(command.chanel, DEC);
  Serial.print(" = ");
  Serial.println(command.value, DEC);

  RF24_send(radio, command);

  return true;
}

void send_ONLINE() {
  RF24_com_t command = {.chanel = CHS::O, .value = 0x1};
  RF24_send(radio, command);
  return;
}
