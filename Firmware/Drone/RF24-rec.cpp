#include "RF24-rec.h"

#include <RF24.h>

#define CH 90 // RF24 chanel
#define ADDR 0xAABBCCDD11LL // RF24 pipe address (aka MAC)

void RF24_init(RF24& radio, HardwareSerial& dbg_ser) {
  if (!radio.begin()) {
    dbg_ser.println("Radio FAIL");
    return;
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, ADDR);
  radio.setChannel(CH);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  dbg_ser.print("Channel noise level: ");
  dbg_ser.println(radio.testCarrier() ? "❌ Busy" : "✅ Free");

  radio.setRetries(5, 15);  // (5 attempts, 15 * 250us delay)
  radio.setAutoAck(true);   // Enable ACK response

  bool ackStatus = radio.isAckPayloadAvailable();
}

bool RF24_command(RF24& radio, RF24_com_t& command) {
  command.chanel = CHS::INVALID;
  command.value = 0x0;

  if (!radio.available()) {
    return;
  }

  /**
   * @brief received data packet
   * [0] - chanel id
   * [1-2] - value
   * [3] - xor checksum
   */
  uint8_t data[4] = {0,0,0,0};

  radio.read(&data, sizeof(data));
  if (data[0] == CHS::O) {
    Serial.print("ONLINE \r");
  }
  else {
    Serial.print(data[0], HEX);
    Serial.print(' ');
    Serial.print(data[1], HEX);
    Serial.print(' ');
    Serial.print(data[2], HEX);
    Serial.print(' ');
    Serial.println(data[3], HEX);
  }

  if (data[3] != (data[0] ^ data[1] ^ data[2])) {
    Serial.println("XOR failed");
    return false;
  }

  command.chanel = data[0];
  command.value = (int16_t)((uint16_t)data[1] << 8 | data[2]);
  return true;
}