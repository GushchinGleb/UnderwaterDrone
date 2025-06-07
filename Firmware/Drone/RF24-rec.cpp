#include "RF24-rec.h"

#include <RF24.h>
#include <SD.h>

#include "chanels.h"
#include "MH-SD.h"

#define CONF_PATH F("rf.txt")

#define CH 90 // RF24 chanel
#define ADDR 0xAABBCCDD11LL // RF24 pipe address (aka MAC)

static uint8_t readRowUInt8(File& file);

static uint64_t readRowUInt64H(File& file);

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

static uint64_t readRowUInt64H(File& file) {
  char s = '\0';
  uint64_t result = 0;
  while (file.available() && s != '\n') {
    s = file.read();
    if ('0' <= s && s <= '9') {
      result <<= 4;
      result |= s - '0';
    } else if ('A' <= s && s <= 'F') {
      result <<= 4;
      result |= s - ('A' - 10);
    }
  }

  return result;
}

void RF24_init(RF24& radio, HardwareSerial& dbg_ser) {
  if (!radio.begin()) {
    dbg_ser.println(F("Radio FAIL"));
    return;
  }

  /**
   * - PA_LEVEL    [0; 3] {MIN, LOW, HIGH, MAX}
   * - PIPE_NUMBER [0; 255]
   * - PIPE_ADDR   [0; 2^64 - 1]
   * - CHANNEL     [0; 255]
   * - DATA_RATE   [0; 2] {1M, 2M, 250K}
   */
  File RF_options = SD.open(CONF_PATH, FILE_READ);

  if (!RF_options) {
    RF_options.close();

    radio.setPALevel(RF24_PA_HIGH);
    radio.openReadingPipe(1, ADDR);
    radio.setChannel(CH);
    radio.setDataRate(RF24_250KBPS);

    SD.remove(CONF_PATH);
    File RF_options = SD.open(CONF_PATH, FILE_WRITE);
    if (!RF_options) {
      Serial.println(F("Failed to open rf.conf"));
      goto SKIP_UPDATE;
    }
    const char data[] PROGMEM = "2\n1\nAABBCCDD11\n90\n2\n";
    RF_options.write(data, sizeof(data) - 1);
    RF_options.close();
  } else {
    const uint8_t pa_level = readRowUInt8(RF_options);
    radio.setPALevel(pa_level < RF24_PA_MAX ? pa_level : RF24_PA_MAX);

    const uint8_t pipe_number = readRowUInt8(RF_options);
    const uint64_t pipe_addr = readRowUInt64H(RF_options);
    radio.openReadingPipe(pipe_number, pipe_addr);

    const uint8_t channel = readRowUInt8(RF_options);
    radio.setChannel(channel);

    const uint8_t data_rate = readRowUInt8(RF_options);
    radio.setDataRate((rf24_datarate_e)(data_rate < RF24_250KBPS ? data_rate : RF24_250KBPS));

    RF_options.close();
  }

SKIP_UPDATE:

  radio.startListening();

  dbg_ser.print(F("Channel "));
  dbg_ser.print(radio.getChannel());
  dbg_ser.print(F(" noise level: "));
  dbg_ser.println(radio.testCarrier() ? F("❌ Busy") : F("✅ Free"));

  radio.setRetries(5, 15);  // (5 attempts, 15 * 250us delay)
  radio.setAutoAck(true);   // Enable ACK response

  bool ackStatus = radio.isAckPayloadAvailable();

  Serial.println(F("RF compete"));
}

bool RF24_command(RF24& radio, RF24_com_t& command) {
  command.chanel = RF_CS::RF_INVALID;
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
  // if (data[0] == RF_CS::RF_O) {
  //   Serial.print("ONLINE \r");
  // }
  // else if (data[0] != RF_CS::RF_INVALID) {
  //   Serial.print(data[0], HEX);
  //   Serial.print(' ');
  //   Serial.print(data[1], HEX);
  //   Serial.print(' ');
  //   Serial.print(data[2], HEX);
  //   Serial.print(' ');
  //   Serial.println(data[3], HEX);
  // }

  if (data[3] != (data[0] ^ data[1] ^ data[2])) {
    // Serial.println("XOR failed");
    return false;
  }

  command.chanel = data[0];
  command.value = (int16_t)((uint16_t)data[1] << 8 | data[2]);
  return true;
}
