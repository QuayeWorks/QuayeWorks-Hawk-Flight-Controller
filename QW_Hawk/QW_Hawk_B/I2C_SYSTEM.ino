/*
 * Possible I2C method to access functions on other STM32
 * 1. Send codes corresponding to other functions
 * Thats it!
 */
 void controlI2C(char val, uint8_t mode)
 {
  if(mode == 18)
  {
    switch (val) 
    {
      case 'a':    // a sent
        I2CScanner(); // scan the I2C for devices
        break;
    }
  }
 }
 void I2CScanner() 
 {
  Serial.println("The following devices are needed:");
  Serial.println("Compass address = 0x" + String(compassAddress,HEX));
  Serial.println("IMU address = 0x" + String(imuAddress,HEX));
  Serial.println("Barometer adress = 0x" + String(barometerAdress,HEX));
  Serial.println("They should appear in the list below.");
  Serial.println("");
  
  uint8_t error, address, done;
  uint16_t nDevices;
  Serial.println("Scanning address 1 till 127...");
  Serial.println("");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    HWire.beginTransmission(address);
    error = HWire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  done = 1;
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
  delay(2000);
}
