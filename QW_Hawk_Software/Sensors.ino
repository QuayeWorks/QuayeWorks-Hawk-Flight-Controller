///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void) {
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  HWire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  HWire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  HWire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  acc_pitch_cal_value  = EEPROM.read(0x16);
  acc_roll_cal_value  = EEPROM.read(0x17);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    //Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  HWire.endTransmission();                                     //End the transmission.
  HWire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
  acc_y = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_x variable.
  acc_x = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_roll = HWire.read() << 8 | HWire.read();                //Read high and low part of the angular data.
  gyro_pitch = HWire.read() << 8 | HWire.read();               //Read high and low part of the angular data.
  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;                                              //Invert the direction of the axis.

  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
    acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                                  //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                                //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                                    //Subtact the manual gyro yaw calibration value.
  }
}



void read_compass() {
  HWire.beginTransmission(compass_address);                     //Start communication with the compass.
  HWire.write(0x03);                                            //We want to start reading at the hexadecimal location 0x03.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.requestFrom(compass_address, 6);                        //Request 6 bytes from the compass.
  compass_y = HWire.read() << 8 | HWire.read();                 //Add the low and high byte to the compass_y variable.
  compass_y *= -1;                                              //Invert the direction of the axis.
  compass_z = HWire.read() << 8 | HWire.read();                 //Add the low and high byte to the compass_z variable.;
  compass_x = HWire.read() << 8 | HWire.read();                 //Add the low and high byte to the compass_x variable.;
  compass_x *= -1;                                              //Invert the direction of the axis.

  //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
  //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
  if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
    compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
    compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
    compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
    compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
    compass_x += compass_offset_x;                              //Add the x-offset to the raw value.
  }

  //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
  //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
  //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
  if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}

//At startup the registers of the compass need to be set. After that the calibration offset and scale values are calculated.
void setup_compass() {
  HWire.beginTransmission(compass_address);                     //Start communication with the compass.
  HWire.write(0x00);                                            //We want to write to the Configuration Register A (00 hex).
  HWire.write(0x78);                                            //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
  HWire.write(0x20);                                            //Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
  HWire.write(0x00);                                            //Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.
  HWire.endTransmission();                                      //End the transmission with the compass.

//Read the calibration values from the EEPROM.
  for (error = 0; error < 6; error ++)compass_cal_values[error] = EEPROM.read(0x10 + error);
  error = 0;
//Calculate the alibration offset and scale values
  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}


//The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}



void read_barometer(void) {
  barometer_counter ++;

  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      HWire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      HWire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.
    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      HWire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      HWire.write(0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the GPS module is setup and read.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gps_setup(void) {

  Serial1.begin(9600);
  delay(250);

  //Disable GPGSV messages by using the ublox protocol.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial1.write(Disable_GPGSV, 11);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the refresh rate to 5Hz by using the ublox protocol.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial1.write(Set_to_5Hz, 14);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the baud rate to 57.6kbps by using the ublox protocol.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
  Serial1.write(Set_to_57kbps, 28);
  delay(200);

  Serial1.begin(57600);
  delay(200);
}

void read_gps(void) {
  while (Serial1.available() && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = Serial1.read();                                                              //Load a new serial byte in the read_serial_byte variable.
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;                                                                                  //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
      digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                                      //Change the LED on the STM32 to indicate GPS reception.
      //Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    //If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            //Add the degrees multiplied by 10.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             //Add the degrees multiplied by 10.
      lat_gps_actual /= 10;                                                                              //Divide everything by 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              //Add the degrees multiplied by 10.
      lon_gps_actual /= 10;                                                                              //Divide everything by 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
      else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
      else longiude_east = 0;                                                                            //When flying west of the prime meridian the longiude_east variable will be set to 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter the number of satillites from the GGA line.

      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
        lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
        lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.

      l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
      l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

      lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
      lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

      //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
      gps_add_counter = 5;                                                                               //Set the gps_add_counter variable to 5 as a count down loop timer
      new_gps_data_counter = 9;                                                                          //Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
      lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
      lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
      new_gps_data_available = 1;                                                                        //Set the new_gps_data_available to indicate that there is new data available.
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;

  }

  //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }

  if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    if (number_used_sats < 8)digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                //Change the LED on the STM32 to indicate GPS reception.
    else digitalWrite(STM32_board_LED, LOW);                                                              //Turn the LED on the STM solid on (LED function is inverted). Check the STM32 schematic.
    gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.

    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    }

    if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
      //GPS stick move adjustments
      if (flight_mode == 3 && takeoff_detected == 1) {
        if (!latitude_north) {
          l_lat_gps_float_adjust += 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //South correction
        }
        else {
          l_lat_gps_float_adjust -= 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //North correction
        }

        if (!longiude_east) {
          l_lon_gps_float_adjust -= (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //West correction
        }

        else {
          l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //East correction
        }
      }

      if (l_lat_gps_float_adjust > 1) {
        l_lat_waypoint ++;
        l_lat_gps_float_adjust --;
      }
      if (l_lat_gps_float_adjust < -1) {
        l_lat_waypoint --;
        l_lat_gps_float_adjust ++;
      }

      if (l_lon_gps_float_adjust > 1) {
        l_lon_waypoint ++;
        l_lon_gps_float_adjust --;
      }
      if (l_lon_gps_float_adjust < -1) {
        l_lon_waypoint --;
        l_lon_gps_float_adjust ++;
      }

      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

      gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.

      //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      //The Proportional part = (float)gps_lat_error * gps_p_gain.
      //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

      //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }

  if (gps_watchdog_timer + 1000 < millis()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
    if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
      flight_mode = 2;                                                                                    //Set the flight mode to 2.
      error = 4;                                                                                          //Output an error.
    }
  }

  if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              //If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
      //Reset the waypoints.
      l_lat_waypoint = 0;
      l_lon_waypoint = 0;
    }
  }
}

