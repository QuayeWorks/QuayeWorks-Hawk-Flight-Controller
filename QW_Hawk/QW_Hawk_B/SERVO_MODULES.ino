void armServoControl(char val, uint8_t mode)
{
  ARM_CONTOL_MODE = true;
  if(mode == 1)
  {
    switch (val) 
    {
      case 'a':    // a sent, move Upward
        armServo.attach(ARM_SERVO_PIN); // Attaches the servo on pin ARM_SERVO_PIN to the servo object
        moveUpward();
        break;
      case 'b':    // b sent, move downward
        armServo.attach(ARM_SERVO_PIN); // Attaches the servo on pin ARM_SERVO_PIN to the servo object
        moveDownward();
        break;
    }
    if (switch1State == LOW || switch2State == LOW)// If the switch state is high stop the servo
   {
    //make sure to create a debounce effect.
     armServo.detach(); // Dettaches the servo on pin ARM_SERVO_PIN
     ARM_CONTOL_MODE = false;
   }
  }
}

void cameraStableization(char val, uint8_t mode)
{
  //bunch of math to stabilize the camera
}

void moveUpward()
{
  armPos += armPos;
  armServo.write(armPos);
  delay(2.56);
  //Serial.println(armPos);
}

void moveDownward() 
{
  armPos -= armPos;
  armServo.write(armPos);
  delay(2.56);
  //Serial.println(armPos);
}
