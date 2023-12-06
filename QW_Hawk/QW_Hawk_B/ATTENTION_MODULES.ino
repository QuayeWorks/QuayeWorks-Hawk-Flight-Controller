void controlBuzzer(char val, uint8_t mode)
{
  if(mode == 2)
  {
    switch (val) 
    {
      case 'a':    // a sent
        lostConnectionTone();
        break;
      case 'b':    // b sent
        noGPSSignal();
        break;
      case 'c':    // c sent
        overCycleTone();
        break;
      case 'd':    // d sent
        droneArmingTone();
        break;
    }
  }
}

void lostConnectionTone() {
  tone(BUZZER_PIN, 1250, 500); // Generate a tone of specified frequency and duration
  delay(700); // Wait for 1 second before playing the next tone
}
void noGPSSignal() {
  tone(BUZZER_PIN, 2500, 500); // Generate a tone of specified frequency and duration
  delay(800); // Wait for 1 second before playing the next tone
}
void overCycleTone() {
  tone(BUZZER_PIN, 750, 1000); // Generate a tone of specified frequency and duration
  delay(900); // Wait for 1 second before playing the next tone
}
void droneArmingTone() {
  tone(BUZZER_PIN, 500, 1500); // Generate a tone of specified frequency and duration
  delay(1000); // Wait for 1 second before playing the next tone
}

void controlLEDs(char val, uint8_t mode)
{
  if (input == 'a')
  {
    // Turn on Green LED usig I2C
    // Send character for this
  }
    if (input == 'b')
  {
    // Turn on Blue LED usig I2C
    // Send character for this
  }
}
