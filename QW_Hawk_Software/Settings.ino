///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the settings that can be set via the transmitter are handled.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Changing the settings is explained in this video:
//https://youtu.be/ys-YpOaA2ME
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void change_settings(void) {
  red_led(HIGH);
  green_led(LOW);
  adjustable_setting_1 = variable_1_to_adjust;
  adjustable_setting_2 = variable_2_to_adjust;
  adjustable_setting_3 = variable_3_to_adjust;

  for (error = 0; error < 150; error ++) {
    delay(20);
    send_telemetry_data();
  }
  error = 0;

  while (channel_6 >= 1900) {
    delayMicroseconds(3700);
    send_telemetry_data();
    if (channel_1 > 1550)adjustable_setting_1 += (float)(channel_1 - 1550) * 0.000001;
    if (channel_1 < 1450)adjustable_setting_1 -= (float)(1450 - channel_1) * 0.000001;
    if (adjustable_setting_1 < 0)adjustable_setting_1 = 0;
    variable_1_to_adjust = adjustable_setting_1;
    
    if (channel_2 > 1550)adjustable_setting_2 += (float)(channel_2 - 1550) * 0.000001;
    if (channel_2 < 1450)adjustable_setting_2 -= (float)(1450 - channel_2) * 0.000001;
    if (adjustable_setting_2 < 0)adjustable_setting_2 = 0;
    variable_2_to_adjust = adjustable_setting_2;

    if (channel_4 > 1550)adjustable_setting_3 += (float)(channel_4 - 1550) * 0.000001;
    if (channel_4 < 1450)adjustable_setting_3 -= (float)(1450 - channel_4) * 0.000001;
    if (adjustable_setting_3 < 0)adjustable_setting_3 = 0;
    variable_3_to_adjust = adjustable_setting_3;
  }
  loop_timer = micros();                                                           //Set the timer for the next loop.
  red_led(LOW);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//This is the return to home step program
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void return_to_home(void) {

  if (flight_mode == 4) {
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 0 - make some basic calculations
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 0) {
      //Is the quadcopter nearby? Then land without returning to home.
      if (abs(lat_gps_home - l_lat_waypoint) < 90 && abs(lon_gps_home - l_lon_waypoint) < 90)return_to_home_step = 3;
      else {
        return_to_home_move_factor = 0.0;
        if (return_to_home_lat_factor == 1 || return_to_home_lon_factor == 1)return_to_home_step = 1;
        //cos(((float)l_lat_gps / 1000000.0)
        if (abs(lat_gps_home - l_lat_waypoint) >= abs(lon_gps_home - l_lon_waypoint)) {
          return_to_home_lon_factor = (float)abs(lon_gps_home - l_lon_waypoint) / (float)abs(lat_gps_home - l_lat_waypoint);
          return_to_home_lat_factor = 1;
        }
        else {
          return_to_home_lon_factor = 1;
          return_to_home_lat_factor = (float)abs(lat_gps_home - l_lat_waypoint) / (float)abs(lon_gps_home - l_lon_waypoint);
        }

        if (ground_pressure - actual_pressure < 170)return_to_home_decrease = 170 - (ground_pressure - actual_pressure);
        else return_to_home_decrease = 0;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 1 increase the altitude to 20 meter above ground level
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 1) {
      if (return_to_home_decrease <= 0)return_to_home_step = 2;
      if (return_to_home_decrease > 0) {
        pid_altitude_setpoint -= 0.035;
        return_to_home_decrease -= 0.035;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 2 - Return to the home position
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 2) {
      if (lat_gps_home == l_lat_waypoint && lon_gps_home == l_lon_waypoint)return_to_home_step = 3;
      if (abs(lat_gps_home - l_lat_waypoint) < 160 && abs(lon_gps_home - l_lon_waypoint) < 160 && return_to_home_move_factor > 0.05)return_to_home_move_factor -= 0.00015;
      else if (return_to_home_move_factor < 0.20)return_to_home_move_factor += 0.0001;

      if (lat_gps_home != l_lat_waypoint) {
        if (lat_gps_home > l_lat_waypoint) l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
        if (lat_gps_home < l_lat_waypoint) l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
      }
      if (lon_gps_home != l_lon_waypoint) {
        if (lon_gps_home > l_lon_waypoint) l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
        if (lon_gps_home < l_lon_waypoint) l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 3 decrease the altitude by increasing the pressure setpoint
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 3) {
      if (pid_altitude_setpoint > actual_pressure + 150)return_to_home_step = 4;
      pid_altitude_setpoint += 0.035;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step - 4 Stop the motors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (return_to_home_step == 4) {
      start = 0;
      return_to_home_step = 5;
    }

  }
}



