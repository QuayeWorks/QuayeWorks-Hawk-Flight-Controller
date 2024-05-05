///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the vertical acceleration is calculated over a longer period via a rotating memory.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vertical_acceleration_calculations(void) {
  acc_z_average_short_rotating_mem_location++;
  if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

  acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
  acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
  acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

  if (acc_z_average_short_rotating_mem_location == 0) {
    acc_z_average_long_rotating_mem_location++;

    if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

    acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
    acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
    acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
  }
  acc_z_average_total = acc_z_average_long_total / 50;


  acc_alt_integrated += acc_total_vector - acc_z_average_total;
  if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
    if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
      if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
      else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void) {

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (pid_roll_setpoint_base > 1508)pid_roll_setpoint = pid_roll_setpoint_base - 1508;
  else if (pid_roll_setpoint_base < 1492)pid_roll_setpoint = pid_roll_setpoint_base - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (pid_pitch_setpoint_base > 1508)pid_pitch_setpoint = pid_pitch_setpoint_base - 1508;
  else if (pid_pitch_setpoint_base < 1492)pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the level and compass calibration procedres are handled.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_compass(void) {
  compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
  red_led(HIGH);                                                             //The red led will indicate that the compass calibration is active.
  green_led(LOW);                                                            //Turn off the green led as we don't need it.
  while (channel_2 < 1900) {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    delayMicroseconds(3700);                                                 //Simulate a 250Hz program loop.
    read_compass();                                                          //Read the raw compass values.
    //In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  compass_calibration_on = 0;                                                //Reset the compass_calibration_on variable.

  //The maximum and minimum values are needed for the next startup and are stored
  for (error = 0; error < 6; error ++) EEPROM.write(0x10 + error, compass_cal_values[error]);

  setup_compass();                                                           //Initiallize the compass and set the correct registers.
  read_compass();                                                            //Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                                        //Set the initial compass heading.

  red_led(LOW);
  for (error = 0; error < 15; error ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }

  error = 0;

  loop_timer = micros();                                                     //Set the timer for the next loop.
}


void calibrate_level(void) {
  level_calibration_on = 1;

  while (channel_2 < 1100) {
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    delay(10);
  }
  red_led(HIGH);
  green_led(LOW);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (error = 0; error < 64; error ++) {
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;

  red_led(LOW);
  if (error < 80) {
    EEPROM.write(0x16, acc_pitch_cal_value);
    EEPROM.write(0x17, acc_roll_cal_value);
    //EEPROM.write(0x10 + error, compass_cal_values[error]);
    for (error = 0; error < 15; error ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }
  angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  loop_timer = micros();                                                           //Set the timer for the next loop.
}




