void intro()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");
}

void printMainMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("//----------------------------Main Menu----------------------------//");
  Serial.println("Press the following keys to test the debugging options available.");
  Serial.println("a.  Test The Arm Servo.");
  Serial.println("b.  Test The Buzzer.");
  Serial.println("c.  Test The LED's.");
  Serial.println("d.  Test The Motors.");
  Serial.println("e.  Test The NRF2401.");
  Serial.println("f.  Test The GPS.");
  Serial.println("g.  Test The IMU.");
  Serial.println("h.  Test The Barometer.");
  Serial.println("i.  Test The Compass.");
  Serial.println("j.  Test The Battery.");
  Serial.println("k.  Test The Receiver.");
  Serial.println("l.  Test The Eprom.");
  Serial.println("m.  Test The SD Card.");
  Serial.println("n.  Test The Ultrasonic Sensors");
  Serial.println("o.  Test The OSD (IF AVAILABLE)");
  Serial.println("p.  Test The Autonomous Mode (IF AVAILABLE)");
  Serial.println("q.  Test Camera Stabililization (IF AVAILABLE)");
  Serial.println("r.  Test The I2C features.");
  Serial.println("Each option may contain further menu features to test.");
}

void printArmServoMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("//----------------------------Arm Servo----------------------------//");
  Serial.println("a.  Raise The Servo.");
  Serial.println("b.  Lower The Buzzer.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printBuzzerMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("//----------------------------Buzzer Menu----------------------------//");
  Serial.println("a.  Play The Lost-Connection Tone.");
  Serial.println("b.  Play The No-GPS-Signal Tone.");
  Serial.println("c.  Play The Over-Cycle Tone.");
  Serial.println("d.  Play The Drone-Arming Tone.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printLEDMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Test The Green LED.");
  Serial.println("b.  Test The Blue LED.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printMotorMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("1.  Test Motor 1 + vibrations. (Middle-Right)");
  Serial.println("2.  Test Motor 2 + vibrations. (Middle-Left)");
  Serial.println("3.  Test Motor 3 + vibrations. (Front-Left)");
  Serial.println("4.  Test Motor 4 + vibrations. (Rear-Right)");
  Serial.println("5.  Test Motor 5 + vibrations. (Front-Right)");
  Serial.println("6.  Test Motor 6 + vibrations. (Rear-Left)");
  Serial.println("7.  Test All The Motors + vibrations.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printNRF2401Menu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Test Transmit Properties.");
  Serial.println("b.  Test Receive Properties.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printGPSMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");
  
  Serial.println("a.  Print The GPS Coordinates.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printIMUMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Setup The IMU.");
  Serial.println("b.  Print The Raw Accelerometer values.");
  Serial.println("c.  Print The Raw Gyro Values.");
  Serial.println("d.  Print The IMU Angles.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printBarometerMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Setup The Barometer.");
  Serial.println("b.  Test The Barometric Pressure.");
  Serial.println("c.  Test The Barometric Altitude.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printCompassMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Setup The Compass.");
  Serial.println("b.  Test The Orientation.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printBatteryMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print The Battery Voltage.");
  Serial.println("b.  Print The Battery Current Usage.");
  Serial.println("c.  Print The Battery Life.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printReceiverMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print The Receiver Signals.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printEpromMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print The Eprom Values.");
  Serial.println("b.  Clear The Eprom Values. (Clears The PID, COMPASS, GYRO And BAROMETER Values");
  Serial.println("0.  Exit To The Main Menu.");
}

void printSDCardMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print The SD Card CSV values.");
  Serial.println("b.  Cleasr The SD Card.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printUltrasonicSensorMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Test The Front Ultrasonic Sensor.");
  Serial.println("b.  Test The Left Ultrasonic Sensor.");
  Serial.println("c.  Test The Right Ultrasonic Sensor.");
  Serial.println("d.  Test The Rear Ultrasonic Sensor.");
  Serial.println("e.  View The Ultrasonicic Vision.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printOSDMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print The Receiver Signals.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printAutonomousMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Test Autonomous Methods.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printCameraMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Stabilize Camera.");
  Serial.println("0.  Exit To The Main Menu.");
}

void printI2CMenu()
{
  Serial.println("QuayeWorks - OS V0.1A");
  Serial.println("Series: HAWK");
  Serial.println("https://github.com/QuayeWorks/QuayeWorks-Hawk-Flight-Controller");

  Serial.println("a.  Print All I2C devices.");
  Serial.println("0.  Exit To The Main Menu.");
}
