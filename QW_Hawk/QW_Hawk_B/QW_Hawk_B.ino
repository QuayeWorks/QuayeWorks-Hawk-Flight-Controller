#include <Servo.h>
#include <Wire.h>
//----------------end Inclusions--------------------//

#define BUZZER_PIN PA8        // Define the buzzer pin

#define SWITCH_1_PIN PA11     // Define the 1st arm switch pin
#define SWITCH_2_PIN PA12     // Define the 2nd arm switch pin

#define ARM_SERVO_PIN PA0     // Define the arm servo pin
#define ROLL_SERVO_PIN PA1    // Define the roll servo pin
#define PITCH_SERVO_PIN PA2   // Define the pitch servo pin
#define YAW_SERVO_PIN PA3     // Define the yaw servo pin

#define SONAR1_TRIG_PIN PB1   // Define the trig pin for sonar 1
#define SONAR1_ECHO_PIN PB3   // Define the echo pin for sonar 1
#define SONAR2_TRIG_PIN PB6   // Define the trig pin for sonar 2
#define SONAR2_ECHO_PIN PB5   // Define the echo pin for sonar 2
#define SONAR3_TRIG_PIN PB7   // Define the trig pin for sonar 3
#define SONAR3_ECHO_PIN PB4   // Define the echo pin for sonar 3
#define SONAR4_TRIG_PIN PB8   // Define the trig pin for sonar 4
#define SONAR4_ECHO_PIN PB12  // Define the echo pin for sonar 4
#define SONAR5_TRIG_PIN PB9   // Define the trig pin for sonar 5
#define SONAR5_ECHO_PIN PB13  // Define the echo pin for sonar 5

#define NRF2401_MISO_PIN PA6  // Define the miso pin for the nrf2401
#define NRF2401_SCK_PIN PA5   // Define the sck pin for the nrf2401
#define NRF2401_CE_PIN PB0    // Define the ce pin for the nrf2401
#define NRF2401_MOSI_PIN PA7  // Define the mosi pin for the nrf2401
#define NRF2401_CSN_PIN PA4   // Define the csn pin for the nrf2401

#define OSD_TX_PIN PA9        // Define the tx pin for the osd
#define OSD_RX_PIN PA10       // Define the rx pin for the osd

#define LED1_PIN PA13         // Define the pin for the 1st led rail
#define LED2_PIN PA14         // Define the pin for the 2nd led rail
#define LED3_PIN PA15         // Define the pin for the 3rd led rail
#define LED4_PIN PB14         // Define the pin for the 4th led rail
#define LED5_PIN PB15         // Define the pin for the 5th led rail
#define LED6_PIN PC13         // Define the pin for the 6th led rail

/*
 * These pins will need to use I2C to control them
 * #define GREEN_LED_PIN PB4  // Define the pin for the green led
 * #define BLUE_LED_PIN PB3   // Define the pin for the blue led
 * #define BATTERY_PIN PA4    // Define the pin for the battery
 * #define MOTOR1 PA0         // Define the pin for the 1st motor
 * #define MOTOR2 PA1         // Define the pin for the 2nd motor
 * #define MOTOR3 PA2         // Define the pin for the 3rd motor
 * #define MOTOR4 PA3         // Define the pin for the 4th motor
 * #define MOTOR5 PA6         // Define the pin for the 5th motor
 * #define MOTOR6 PA7         // Define the pin for the 6th motor
 * #define MOTOR7 PB0         // Define the pin for the 7th motor
 * #define MOTOR8 PB1         // Define the pin for the 8th motor
 * #define SDCARD_CMD PB15    // Define the cmd pin for the SD CARD
 * #define SDCARD_DATA0 PB14  // Define the data0 pin for the SD CARD
 * #define SDCARD_CLK PB13    // Define the clk pin for the SD CARD
 * #define SDCARD_CD PB12     // Define the cd pin for the SD CARD
*/
//----------------end definitions-------------------//

Servo armServo; // create servo object to control a servo
TwoWire HWire (2, I2C_FAST_MODE);

uint8_t menuState = 0;
uint8_t armPos = 0;             // variable to store the servo position
uint8_t switch1State;           // 
uint8_t switch2State;           // 
uint8_t barometerAdress = 0x77; // The Barometer I2C address.
uint8_t imuAddress = 0x68;      // The IMU I2C address.
uint8_t compassAddress = 0x1E;  // The Compass I2C address.

bool DEBUG_MODE = true;
bool ARM_CONTOL_MODE = false;
bool printIntro = true;

char input = ' ';
//----------------end variables---------------------//

void setup() {
  pinMode(ARM_SERVO_PIN, OUTPUT); // Set Arm Servo pin as an output.
  pinMode(SWITCH_2_PIN, INPUT);   // Set the arm bottom switch as an input. (SW2)
  pinMode(SWITCH_1_PIN, INPUT);   // Set the arm top switch as an input. (SW1)
  pinMode(BUZZER_PIN, OUTPUT);    // Set the buzzer input in pwm mode.
  
  Serial.begin(115200); // Start the serial at a BAUD rate of 115200
  armServo.attach(ARM_SERVO_PIN); // Attaches the servo on pin PA0 to the servo object
}  

void loop() 
{
  if(ARM_CONTOL_MODE == true)
  {
    switch1State = digitalRead(SWITCH_1_PIN);
    //Serial.println("Switch 1 State: " + String(switch1State));
    switch2State = digitalRead(SWITCH_2_PIN);
    //Serial.println("Switch 2 State: " + String(switch2State));
  }
  if (DEBUG_MODE == true)
  {
    if(printIntro)
    {
      printMainMenu();
      printIntro = false;
    }
    if (Serial.available() > 0)
    {
      char input = Serial.read(); // Read the input from serial monitor
      armServoControl(input, menuState);
      controlBuzzer(input, menuState);
      controlLEDs(input, menuState);
      cameraStableization(input, menuState);
      controlI2C(input, menuState);
      
      if (input == '0')
      {
        menuState = 0;
        printMainMenu();
      }
      if (menuState == 0)
      {
        if(input == 'a')
        {
          menuState = 1;
          printArmServoMenu();
        }
        if(input == 'b')
        {
          menuState = 2;
          printBuzzerMenu();
        }
        if(input == 'c')
        {
          menuState = 3;
          printLEDMenu();
        }
        if(input == 'd')
        {
          menuState = 4;
          printMotorMenu();
        }
        if(input == 'e')
        {
          menuState = 5;
          printNRF2401Menu();
        }
        if(input == 'f')
        {
          menuState = 6;
          printGPSMenu();
        }
        if(input == 'g')
        {
          menuState = 7;
          printIMUMenu();
        }
        if(input == 'h')
        {
          menuState = 8;
          printBarometerMenu();
        }
        if(input == 'i')
        {
          menuState = 9;
          printCompassMenu();
        }
        if(input == 'j')
        {
          menuState = 10;
          printBatteryMenu();
        }
        if(input == 'k')
        {
          menuState = 11;
          printReceiverMenu();
        }
        if(input == 'l')
        {
          menuState = 12;
          printEpromMenu();
        }
        if(input == 'm')
        {
          menuState = 13;
          printSDCardMenu();
        }
        if(input == 'n')
        {
          menuState = 14;
          printUltrasonicSensorMenu();
        }
        if(input == 'o')
        {
          menuState = 15;
          printOSDMenu();
        }
        if(input == 'p')
        {
          menuState = 16;
          printAutonomousMenu();
        }
        if(input == 'q')
        {
          menuState = 17;
          printCameraMenu();
        }
        if(input == 'r')
        {
          menuState = 18;
          printI2CMenu();
        }
      }
    }
  }
}
