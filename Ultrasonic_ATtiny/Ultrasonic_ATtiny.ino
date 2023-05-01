#include <Wire.h>

#define SLAVE_ADDRESS 0x08

void setup() {
  Wire.begin();        // join i2c bus
  Serial.begin(9600);  // start serial for debugging
}

void loop() {
  Wire.beginTransmission(SLAVE_ADDRESS); // transmit to device with address SLAVE_ADDRESS
  Wire.write("Hello I2C");               // send a string
  Wire.endTransmission();                // stop transmitting

  delay(500);

  Wire.requestFrom(SLAVE_ADDRESS, 6);    // request 6 bytes from device with address SLAVE_ADDRESS

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read();    // receive a byte as character
    Serial.print(c);         // print the character received
  }

  delay(500);
}
