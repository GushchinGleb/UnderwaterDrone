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

  send_from_serial();

  if (t > send_tmg) {
    send_tmg = t + 250;
    send_ONLINE();
  }
}

bool send_from_serial() {
  if (!Serial.available()) {
    return false;
  }

  int character = Serial.read();

  Serial.print((char)character);

  static RF24_com_t command;

  /**
   * 0 - wait for symbol
   * 1 - wait for value
   *
   * 0 -> 1 -> 0 ...
   */
  static uint8_t machine_state = 0;

  /**
   * 0 - positive
   * 1 - negative
   */
  static int16_t machine_state_neg = 0;

  #define SET_STATE(state) command.chanel = CHS::state; machine_state = 1; machine_state_neg = 0; command.value = 0; break

  switch (machine_state) {
  case 0: // wait for chanel id symbol
    switch (character) {
    case 'X': SET_STATE(X); // left motor
    case 'Y': SET_STATE(Y); // right motor
    case 'l': SET_STATE(l); // left ballast
    case 'r': SET_STATE(r); // right ballast
    case 'O': SET_STATE(O); // online signal
    case 'm': SET_STATE(m); // online signal
    case '\r':
    case '\n':
      Serial.println("resset");
      machine_state = 0;
      break;
    default: break; // skip not described symbols, without errors errors.
    }
    break;

  case 1: // wait for channel value
    if (character == '-') {
      machine_state_neg = 1; // number is negative
    } else if (character >= '0' && character <= '9') {
      command.value *= 10;
      command.value += character - '0';
    } else if (character == '\n' || character == '\r') {
      if (machine_state_neg) command.value = -command.value;

      RF24_send(radio, command);
      Serial.print("Send: ch: ");
      Serial.print(command.chanel);
      Serial.print(" value: ");
      Serial.println(command.value);

      machine_state = 0; // wait for a new command
      return true;
    }
    break;
  default:
    machine_state = 0; // error wait for a command character
    break;
  }

  return false;
}

void send_ONLINE() {
  RF24_com_t command = {.chanel = CHS::O, .value = 0x1};
  RF24_send(radio, command);
  return;
}
