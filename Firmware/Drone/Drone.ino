#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 9;
const int CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);  // Create RF24 object

const byte address[6] = "00001";  // Address for communication
int dataToSend = 123;  // Sample data to send

void setup() {
  Serial.begin(9600);
  radio.begin();  // Start the radio
  radio.setPALevel(RF24_PA_LOW);  // Set power level
  radio.openWritingPipe(address);  // Open the writing pipe
  radio.stopListening();  // Stop listening to be able to transmit
}

void loop() {
  bool success = radio.write(&dataToSend, sizeof(dataToSend));
  if (success) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
  }
  delay(1000);  // Send data every second
}
