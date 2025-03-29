#include "gy-neo6mv2.h" // GPS sensor
#include "hmc5883l.h"   // magnitometer (compas)
#include "motor.h"      // drivers for the motors
#include "mpu6050.h"    // accelerometr

#include "fast-math.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <nRF24L01.h>
#include <RF24.h>

// #define DBG_SER (Serial) // debug serial instance
// #define DBG_SPEED (9600) // debug serial speed

// static uint32_t t; // time from the start in milliseconds
// static uint32_t e_p; // event: print data

// static gy_neo6mv2_data_t gy_data; // gps data

// void printData(const int16_t mpu[7], const int16_t hmc[3]);

// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);

//   DBG_SER.begin(DBG_SPEED);
//   Wire.begin();

//   gy_neo6mv2_init(gy_data);
//   hmc5883_init();
//   motor_init();
//   mpu6050_init();

//   t = millis();
//   e_p = t;

//   digitalWrite(LED_BUILTIN, LOW);
// }

// void loop() {
//   static uint8_t motor_status = 0;
//   t = millis();

//   if (t > e_p) {
//     e_p = t + 2000; // + 2 seconds

//     switch (motor_status) {
//     case 0:
//       digitalWrite(23, LOW);
//       digitalWrite(25, LOW);
//       break;
//     case 1:
//       digitalWrite(25, HIGH);
//       break;
//     case 2:
//       digitalWrite(23, LOW);
//       digitalWrite(25, LOW);
//       break;
//     case 3:
//       digitalWrite(25, HIGH);
//       break;
//     default: motor_status = 0; break;
//     }
//     DBG_SER.print(F("Motor status: "));
//     DBG_SER.println(motor_status);
//     ++motor_status;
//     if (motor_status >= 4) {
//       motor_status = 0;
//     }
//   }

//   // while (GY_SERIAL.available()) {
//   //   DBG_SER.write(GY_SERIAL.read());
//   // }
// }

// void printData(const int16_t mpu[7], const int16_t hmc[3]) {
//   for (uint8_t i = 0; i < 7; ++i) {
//     DBG_SER.print(mpu[i]);
//     DBG_SER.print(' ');
//   }
//   DBG_SER.println("");

//   for (uint8_t i = 0; i < 3; ++i) {
//     DBG_SER.print(mpu[i]);
//     DBG_SER.print(' ');
//   }

//   DBG_SER.println("");
// }

// void printGY(const gy_neo6mv2_data_t& gy_data) {
//   DBG_SER.print(F("Time: "));
//   DBG_SER.print(gy_data.time[0]); DBG_SER.print(":");
//   DBG_SER.print(gy_data.time[1]); DBG_SER.print(":");
//   DBG_SER.println(gy_data.time[2]);

//   DBG_SER.print(F("Latitude:  ")); DBG_SER.println(gy_data.lat, 6);
//   DBG_SER.print(F("Longitude: ")); DBG_SER.println(gy_data.lon, 6);
//   DBG_SER.print(F("Fix:       ")); DBG_SER.println(gy_data.fix);
//   DBG_SER.print(F("Altitude:  ")); DBG_SER.print(gy_data.alt); DBG_SER.println(" m");
//   DBG_SER.print(F("Sea Level: ")); DBG_SER.print(gy_data.sea); DBG_SER.println(" m");
// }

#include <SPI.h>
#include <RF24.h>

#define CE_PIN  42   // CE pin for nRF24L01+
#define CSN_PIN 40  // CSN pin for nRF24L01+

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for serial connection
    
    Serial.println("Initializing nRF24L01+ module...");
    if (!radio.begin()) {
        Serial.println("[ERROR] RF24 module not detected!");
        while (1); // Halt if module is not found
    }
    Serial.println("[OK] RF24 module initialized.");

    // Print RF24 settings
    Serial.print("Channel: "); Serial.println(radio.getChannel());
    Serial.print("Data Rate: ");
    switch (radio.getDataRate()) {
        case RF24_250KBPS: Serial.println("250 Kbps"); break;
        case RF24_1MBPS:   Serial.println("1 Mbps"); break;
        case RF24_2MBPS:   Serial.println("2 Mbps"); break;
    }
    Serial.print("PA Level: ");
    switch (radio.getPALevel()) {
        case RF24_PA_MIN: Serial.println("MIN"); break;
        case RF24_PA_LOW: Serial.println("LOW"); break;
        case RF24_PA_HIGH: Serial.println("HIGH"); break;
        case RF24_PA_MAX: Serial.println("MAX"); break;
    }
}

void scanNetwork() {
    Serial.println("Scanning RF24 network (Channels 1-125)...");
    Serial.println("Channel  | Signal Strength");
    Serial.println("---------------------------");
    
    for (int ch = 1; ch <= 125; ch++) {
        radio.setChannel(ch);
        delayMicroseconds(110);
        radio.startListening();
        delay(5);
        radio.stopListening();
        
        if (radio.testCarrier()) {
            Serial.print("   "); Serial.print(ch);
            Serial.println("  | ACTIVE");
        }
    }
    Serial.println("Scan complete!");
}

void loop() {
    scanNetwork();
    delay(5000); // Wait 5 seconds before next scan
}
