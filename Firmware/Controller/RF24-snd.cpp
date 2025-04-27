#include "RF24-snd.h"

#define CH 90 // RF24 chanel
#define ADDR 0xAABBCCDD11LL // RF24 pipe address (aka MAC)

void RF24_init(RF24& radio, HardwareSerial& dbg_ser) {
  if (!radio.begin()) {
    dbg_ser.println("Radio FAIL");
    return;
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(ADDR);
  radio.setChannel(CH);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  dbg_ser.print("Channel noise level: ");
  dbg_ser.println(radio.testCarrier() ? "❌ Busy" : "✅ Free");
  
  radio.setRetries(5, 15);  // (5 attempts, 15 * 250us delay)
  radio.setAutoAck(true);   // Enable ACK response

  bool ackStatus = radio.isAckPayloadAvailable();
}

void RF24_send(RF24& radio, RF24_com_t& command) {
  /**
   * @brief received data packet
   * [0] - chanel id
   * [1-2] - value
   * [3] - xor checksum
   */
  uint8_t data[4] = {
    command.chanel,
    (uint8_t)(command.value >> 8),
    (uint8_t)command.value,
    0x0
  };
  data[3] = data[0] ^ data[1] ^ data[2];
  
  bool success = radio.write(data, sizeof(data)); // Send over RF24
  return;
}