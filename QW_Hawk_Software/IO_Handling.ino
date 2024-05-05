///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void handler_channel_1(void) {
  measured_time = TIMER2_BASE->CCR1 - measured_time_start;
  if (measured_time < 0)measured_time += 0xFFFF;
  measured_time_start = TIMER2_BASE->CCR1;
  if (measured_time > 3000) {
    channel_select_counter = 0;
    receiver_watchdog = 0;
    if (error == 8 && start == 2)error = 0;
  }
  else channel_select_counter++;

  if (channel_select_counter == 1)channel_1 = measured_time;
  if (channel_select_counter == 2)channel_2 = measured_time;
  if (channel_select_counter == 3)channel_3 = measured_time;
  if (channel_select_counter == 4)channel_4 = measured_time;
  if (channel_select_counter == 5)channel_5 = measured_time;
  if (channel_select_counter == 6)channel_6 = measured_time;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. The LEDs on the flip 32 are inverted. That is why a Flip32 test is needed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void red_led(int8_t level) {
  if (flip32)digitalWrite(PB4, !level);    //If a Flip32 is used invert the output.
  else digitalWrite(PB4, level);           //When using the BluePill the output should not be inverted.
}
void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);    //If a Flip32 is used invert the output.
  else digitalWrite(PB3, level);           //When using the BluePill the output should not be inverted.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void) {
  if (error >= 100) red_led(HIGH);                                                         //When the error is 100 the LED is always on.
  else if (error_timer < millis()) {                                                       //If the error_timer value is smaller that the millis() function.
    error_timer = millis() + 250;                                                          //Set the next error_timer interval at 250ms.
    if (error > 0 && error_counter > error + 3) error_counter = 0;                         //If there is an error to report and the error_counter > error +3 reset the error.
    if (error_counter < error && error_led == 0 && error > 0) {                            //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      red_led(HIGH);                                                                       //Turn the LED on.
      error_led = 1;                                                                       //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      red_led(LOW);                                                                        //Turn the LED off.
      error_counter++;                                                                     //Increment the error_counter variable by 1 to keep trach of the flashes.
      error_led = 0;                                                                       //Set the LED flag to indicate that the LED is off.
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the flight mode LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                      //If the error_timer value is smaller that the millis() function.
    flight_mode_timer = millis() + 250;                                                    //Set the next error_timer interval at 250ms.
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      green_led(HIGH);                                                                     //Turn the LED on.
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      green_led(LOW);                                                                      //Turn the LED off.
      flight_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      flight_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;

  //TIMER2_BASE->CCER |= TIMER_CCER_CC1P;    //Detect falling edge.
  TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P; //Detect rising edge.
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;

  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part sends the telemetry data to the ground station.
//The output for the serial monitor is PB0. Protocol is 1 start bit, 8 data bits, no parity, 1 stop bit.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_telemetry_data(void) {
  telemetry_loop_counter++;                                                                 //Increment the telemetry_loop_counter variable.
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'J';                                //Send a J as start signature.
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'B';                                //Send a B as start signature.
  if (telemetry_loop_counter == 3) {
    check_byte = 0;
    telemetry_send_byte = error;                              //Send the error as a byte.
  }
  if (telemetry_loop_counter == 4)telemetry_send_byte = flight_mode + return_to_home_step;                        //Send the flight mode as a byte.
  if (telemetry_loop_counter == 5)telemetry_send_byte = battery_voltage * 10;               //Send the battery voltage as a byte.
  if (telemetry_loop_counter == 6) {
    telemetry_buffer_byte = temperature;                                                    //Store the temperature as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the temperature variable.
  }
  if (telemetry_loop_counter == 7)telemetry_send_byte = telemetry_buffer_byte >> 8;         //Send the last 8 bytes of the temperature variable.
  if (telemetry_loop_counter == 8)telemetry_send_byte = angle_roll + 100;                   //Send the roll angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 9)telemetry_send_byte = angle_pitch + 100;                  //Send the pitch angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 10)telemetry_send_byte = start;                             //Send the error as a byte.
  if (telemetry_loop_counter == 11) {
    if (start == 2) {                                                                       //Only send the altitude when the quadcopter is flying.
      telemetry_buffer_byte = 1000 + ((float)(ground_pressure - actual_pressure) * 0.117);  //Calculate the altitude and add 1000 to prevent negative numbers.
    }
    else {
      telemetry_buffer_byte = 1000;                                                         //Send and altitude of 0 meters if the quadcopter isn't flying.
    }
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the altitude variable.
  }
  if (telemetry_loop_counter == 12)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the altitude variable.

  if (telemetry_loop_counter == 13) {
    telemetry_buffer_byte = 1500 + takeoff_throttle;                                        //Store the take-off throttle as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the take-off throttle variable.
  }
  if (telemetry_loop_counter == 14)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the take-off throttle variable.
  if (telemetry_loop_counter == 15) {
    telemetry_buffer_byte = angle_yaw;                                                      //Store the compass heading as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the compass heading variable.
  }
  if (telemetry_loop_counter == 16)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the compass heading variable.
  if (telemetry_loop_counter == 17)telemetry_send_byte = heading_lock;                      //Send the heading_lock variable as a byte.
  if (telemetry_loop_counter == 18)telemetry_send_byte = number_used_sats;                  //Send the number_used_sats variable as a byte.
  if (telemetry_loop_counter == 19)telemetry_send_byte = fix_type;                          //Send the fix_type variable as a byte.
  if (telemetry_loop_counter == 20) {
    telemetry_buffer_byte = l_lat_gps;                                                      //Store the latitude position as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the latitude position variable.
  }
  if (telemetry_loop_counter == 21)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 24) {
    telemetry_buffer_byte = l_lon_gps;                                                      //Store the longitude position as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the longitude position variable.
  }
  if (telemetry_loop_counter == 25)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 26)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 27)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send the next 8 bytes of the longitude position variable.

  if (telemetry_loop_counter == 28) {
    telemetry_buffer_byte = adjustable_setting_1 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 1.
  }
  if (telemetry_loop_counter == 29)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 1.
  if (telemetry_loop_counter == 30) {
    telemetry_buffer_byte = adjustable_setting_2 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 2.
  }
  if (telemetry_loop_counter == 31)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 2.
  if (telemetry_loop_counter == 32) {
    telemetry_buffer_byte = adjustable_setting_3 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 3.
  }
  if (telemetry_loop_counter == 33)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 3.


  if (telemetry_loop_counter == 34)telemetry_send_byte = check_byte;                        //Send the check-byte.


  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == 125)telemetry_loop_counter = 0;                             //After 125 loops reset the telemetry_loop_counter variable

  //Send the telemetry_send_byte via the serial protocol via ouput PB0.
  //Send a start bit first.
  if (telemetry_loop_counter <= 34) {
    check_byte ^= telemetry_send_byte;
    GPIOB_BASE->BSRR = 0b1 << 16;                                                             //Reset output PB0 to 0 to create a start bit.
    delayMicroseconds(104);                                                                   //Delay 104us (1s/9600bps)
    for (telemetry_bit_counter = 0; telemetry_bit_counter < 8; telemetry_bit_counter ++) {    //Create a loop fore every bit in the
      if (telemetry_send_byte >> telemetry_bit_counter & 0b1) GPIOB_BASE->BSRR = 0b1 << 0;    //If the specific bit is set, set output PB0 to 1;
      else GPIOB_BASE->BSRR = 0b1 << 16;                                                      //If the specific bit is not set, reset output PB0 to 0;
      delayMicroseconds(104);                                                                 //Delay 104us (1s/9600bps)
    }
    //Send a stop bit
    GPIOB_BASE->BSRR = 0b1 << 0;                                                              //Set output PB0 to 1;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;                              //For starting the motors: throttle low and yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //When yaw stick is back in the center position start the motors (step 2).
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                             //Register the pressure at ground level for altitude calculations.
    course_lock_heading = angle_yaw;                                               //Set the current compass heading as the course lock heading.
    acc_total_vector_at_start = acc_total_vector;                                  //Register the acceleration when the quadcopter is started.
    if(number_used_sats >= 5){
      lat_gps_home = l_lat_gps;
      lon_gps_home = l_lon_gps;
      home_point_recorded = 1;
    }
    else home_point_recorded = 0;
    start = 2;                                                                     //Set the start variable to 2 to indicate that the quadcopter is started.
    acc_alt_integrated = 0;                                                        //Reset the integrated acceleration value.
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {        //If the manual hover throttle is used and valid (between 1400us and 1600us pulse).
      takeoff_throttle = manual_takeoff_throttle - 1500;                           //Use the manual hover throttle.
      takeoff_detected = 1;                                                        //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
      //Reset the PID controllers for a smooth take-off.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {                                            //If the manual hover throttle value is invalid.
      error = 5;                                                                   //Error = 5.
      takeoff_throttle = 0;                                                        //No hover throttle compensation.
      start = 0;                                                                   //Set the start variable to 0 to stop the motors.
    }
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff_detected = 0;                                                          //Reset the auto take-off detection.
  }

  if (takeoff_detected == 0 && start == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (channel_3 > 1480 && throttle < 1750) throttle++;                           //When the throttle is half way or higher, increase the throttle.
    if (throttle == 1750)error = 6;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
    if (channel_3 <= 1480) {                                                       //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {        //A take-off is detected when the quadcopter is accelerating.
      takeoff_detected = 1;                                                        //Set the take-off detected variable to 1 to indicate a take-off.
      pid_altitude_setpoint = ground_pressure - 22;                                //Set the altitude setpoint at groundlevel + approximately 2.2 meters.
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;  //If the automated throttle is between 1400 and 1600us during take-off, calculate take-off throttle.
      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //No take-off throttle is calculated.
        error = 7;                                                                 //Show error 7 on the red LED.
      }
    }
  }
}

