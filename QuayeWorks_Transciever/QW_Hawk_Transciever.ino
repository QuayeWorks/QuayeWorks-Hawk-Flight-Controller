// By QuayeWorks
///////////////////////////////////////////////////////////////////////////////////////
// Terms of Use and Warranty Disclaimer under GNU General Public License (GPL) version 3.0
///////////////////////////////////////////////////////////////////////////////////////
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

#include <LiquidCrystal.h>
#include <EEPROM.h>

// Declare global variables
uint8_t receive_buffer[50], receive_buffer_counter, receive_byte_previous, receive_start_detect;
uint8_t check_byte, temp_byte;
uint8_t error, alarm_sound, flight_mode;
uint8_t telemetry_lost;
uint8_t alarm_silence;
uint8_t start, flight_timer_start;
uint8_t hours, minutes, seconds;
uint8_t heading_lock;
uint8_t number_used_sats;
uint8_t fix_type, max_speed_from_eeprom, speed_kmph, max_speed, speed_loop_counter;
uint16_t speed_buffer[5];

int8_t page, previous_page;

uint32_t last_receive, next_sound, flight_timer, flight_timer_previous, flight_timer_from_start, flight_time_from_eeprom;
uint32_t hours_flight_time, minutes_flight_time, seconds_flight_time;
int32_t l_lat_gps, l_lon_gps, l_lat_gps_previous, l_lon_gps_previous;
float lat_distance, lon_distance;

int16_t temperature, button_push, button_store, roll_angle, pitch_angle;
int16_t altitude_meters, max_altitude_meters, max_altitude_from_eeprom;
uint16_t key_press_timer;
int16_t takeoff_throttle;
uint16_t actual_compass_heading;

float battery_voltage, adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

byte led;

// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define BUZZER_PIN 2

void initializeTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  OCR2A = 249;
  TCCR2A |= (1 << WGM21);
}

void setup() {
  initializeTimer2();
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);  // Assume BUZZER_PIN is defined as 2
  lcd.begin(16, 2);
  displayWelcomeMessage();
  performStartupTone();

  initializeVariables();
  readEEPROMSettings();
}

void displayWelcomeMessage() {
  lcd.setCursor(4, 0);
  lcd.print("QuayeWorks");
  lcd.setCursor(3, 1);
  lcd.print("Transciever");
}

void performStartupTone() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN, LOW);
    delay(i < 2 ? 50 : 2500);
  }
}

void initializeVariables() {
  button_store = -1;
  lcd.clear();
  telemetry_lost = 2;
}

void readEEPROMSettings() {
  flight_time_from_eeprom = EEPROM_readLong(0x00);
  max_altitude_from_eeprom = EEPROM_readInt(0x04);
  max_speed_from_eeprom = EEPROM.read(0x06);
  max_speed = max_speed_from_eeprom;
}

void loop() {
  resetButtonIfPressedLong();
  handleButtonInteractions();
  handleFlightAndTelemetry();
  handleDisplayUpdates();
}

void resetButtonIfPressedLong() {
  if (key_press_timer > 0) {
    key_press_timer = 0;
    button_store = -1;
  }
}

void handleButtonInteractions() {
  if (button_store != -1) {
    buttonDebounce();
    processButtonPress();
  }
}

void buttonDebounce() {
  while (analogRead(0) < 1000) {
    delay(10);
    if (key_press_timer < 200) key_press_timer++;
    if (key_press_timer == 200) {
      triggerBuzzerSequence();
    }
  }
}

void processButtonPress() {
  if (key_press_timer < 200) {
    triggerBuzzerShort();
  }
  if (button_store != -1 && key_press_timer < 200) {
    navigatePages();
  }
}

void navigatePages() {
  if (button_store < 300 && button_store > 200) page--; // Down
  if (button_store < 150 && button_store > 50) page++; // Up
  if (button_store < 700 && button_store > 600) page = 0; // Select
  page = constrain(page, 0, 7); // Constrain page numbers to valid range
  button_store = -1;
}

void triggerBuzzerSequence() {
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  delay(500);
}

void triggerBuzzerShort() {
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
}

void handleFlightAndTelemetry() {
  if (start > 1 && flight_timer_start == 0) {
    startFlightTimer();
  }
  if (start == 0 && flight_timer_start == 1) {
    storeFlightData();
  }
  readSerialTelemetry();
  checkTelemetryTimeout();
  handleErrorsAndAlarms();
}

void startFlightTimer() {
  flight_timer_from_start = millis();
  flight_timer = millis() - flight_timer_previous;
  flight_timer_start = 1;
}

void storeFlightData() {
  updateMaxValues();
  storeEEPROMData();
  flight_timer_previous = millis() - flight_timer;
  flight_timer_start = 0;
}

void updateMaxValues() {
  if (max_altitude_meters > max_altitude_from_eeprom) {
    max_altitude_from_eeprom = max_altitude_meters;
    EEPROM_writeInt(0x04, max_altitude_from_eeprom);
  }
  if (max_speed > max_speed_from_eeprom) {
    max_speed_from_eeprom = max_speed;
    EEPROM_write(0x06, max_speed_from_eeprom);
  }
}

void storeEEPROMData() {
  flight_time_from_eeprom += (millis() - flight_timer_from_start) / 1000;
  EEPROM_writeLong(0x00, flight_time_from_eeprom);
}

void readSerialTelemetry() {
  if (Serial.available()) {
    processDataFromSerial();
  }
}

void checkTelemetryTimeout() {
  if (last_receive + 3000 < millis() && receive_start_detect && telemetry_lost == 0) {
    telemetry_lost = 1;
    lcd.clear();
    receive_start_detect = 0;
    page = 100; // Lost telemetry page
  }
}

void handleErrorsAndAlarms() {
  if (error && alarm_sound == 0) {
    alarm_sound = 1;
    page = 100 + error;
  }
  if (error == 0 && alarm_sound) {
    alarm_sound = 0;
    page = 0;
  }
  if ((telemetry_lost == 1 || alarm_sound == 1) && next_sound < millis()) {
    triggerAlarmSound();
  }
}

void triggerAlarmSound() {
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  next_sound = millis() + 1000;
}

void handleDisplayUpdates() {
  if (page != previous_page) {
    lcd.clear();
    previous_page = page;
  }
  updateLCD();
}

void updateLCD() {
    // Clear the display only when changing pages to prevent flickering
    if (page != previous_page) {
        lcd.clear();
        previous_page = page;
    }

    // Switch statement to select the display based on the current page number
    switch (page) {
        case 0:
            displayFlightInfo();
            break;
        case 1:
            displaySpeedInfo();
            break;
        case 2:
            displayGPSInfo();
            break;
        case 3:
            displayAltitudeInfo();
            break;
        case 4:
            displayRollPitchInfo();
            break;
        case 5:
            displayFlightTimeInfo();
            break;
        case 6:
            displaySettingsInfo();
            break;
        case 7:
            displayTakeOffThrottleInfo();
            break;
        default:
            if (page >= 100) {
                displayErrorInfo(page - 100); // Assuming error pages start from 100
            } else {
                displayDefaultInfo();
            }
            break;
    }
}

void displayTakeOffThrottleInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Take-off thr:");
    lcd.setCursor(0, 1);
    lcd.print(takeoff_throttle);
}

// Implementation of the display functions like displayFlightInfo(), displaySpeedInfo(), etc.
void displayFlightInfo() {
    lcd.setCursor(0, 0);
    if (flight_mode <= 3) {
        lcd.print("M");
        lcd.print(flight_mode);
    } else {
        lcd.print("R");
        lcd.print(flight_mode - 4);
    }

    lcd.setCursor(5, 0);
    lcd.print("Bat:");
    if (battery_voltage < 10) lcd.print("0");
    lcd.print(battery_voltage, 1);
    lcd.print("V");

    lcd.setCursor(0, 1);
    lcd.print("Alt:");
    if (altitude_meters < 0) lcd.print("-");
    else lcd.print("+");
    if (altitude_meters < 100) lcd.print("0");
    if (altitude_meters < 10) lcd.print("0");
    lcd.print(altitude_meters);
    lcd.print("m");
}

void displaySpeedInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    if (speed_kmph < 100) lcd.print("0");
    if (speed_kmph < 10) lcd.print("0");
    lcd.print(speed_kmph);
    lcd.print("kph");

    lcd.setCursor(0, 1);
    lcd.print("Max: ");
    if (max_speed < 100) lcd.print("0");
    if (max_speed < 10) lcd.print("0");
    lcd.print(max_speed);
    lcd.print("kph");
}

void displayGPSInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.print(l_lat_gps);
    lcd.setCursor(0, 1);
    lcd.print("Lon:");
    lcd.print(l_lon_gps);
}

void displayAltitudeInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Altitude:");
    if (altitude_meters < 1000) lcd.print("0");
    if (altitude_meters < 100) lcd.print("0");
    if (altitude_meters < 10) lcd.print("0");
    lcd.print(altitude_meters);
    lcd.print("m");

    lcd.setCursor(0, 1);
    lcd.print("Max Alt:");
    if (max_altitude_from_eeprom < 1000) lcd.print("0");
    if (max_altitude_from_eeprom < 100) lcd.print("0");
    if (max_altitude_from_eeprom < 10) lcd.print("0");
    lcd.print(max_altitude_from_eeprom);
    lcd.print("m");
}

void displayRollPitchInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Roll: ");
    if (roll_angle < 10 && roll_angle > -10) lcd.print("0");
    lcd.print(roll_angle);
    lcd.print((char)223); // degree symbol

    lcd.setCursor(0, 1);
    lcd.print("Pitch: ");
    if (pitch_angle < 10 && pitch_angle > -10) lcd.print("0");
    lcd.print(pitch_angle);
    lcd.print((char)223); // degree symbol
}

void displayFlightTimeInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Flight Time");
    long total_seconds = flight_time_from_eeprom;
    int hours = total_seconds / 3600;
    int minutes = (total_seconds % 3600) / 60;
    int seconds = total_seconds % 60;

    lcd.setCursor(0, 1);
    lcd.print("Hrs:");
    if (hours < 10) lcd.print("0");
    lcd.print(hours);
    lcd.print(" Min:");
    if (minutes < 10) lcd.print("0");
    lcd.print(minutes);
    lcd.print(" Sec:");
    if (seconds < 10) lcd.print("0");
    lcd.print(seconds);
}

void displaySettingsInfo() {
    lcd.setCursor(0, 0);
    lcd.print("Settings:");
    lcd.setCursor(0, 1);
    lcd.print("1:");
    lcd.print(adjustable_setting_1);
    lcd.print(" 2:");
    lcd.print(adjustable_setting_2);
    lcd.print(" 3:");
    lcd.print(adjustable_setting_3);
}

void displayDefaultInfo() {
    lcd.setCursor(0, 0);
    lcd.print("No data");
    lcd.setCursor(0, 1);
    lcd.print("available");
}

void displayErrorInfo(int errorCode) {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(errorCode);
    lcd.setCursor(0, 1);
    switch (errorCode) {
        case 1:
            lcd.print("Battery LOW!");
            break;
        case 5:
            lcd.print("Loop time exc.");
            break;
        case 6:
            lcd.print("Take-off error");
            break;
        case 10:
            lcd.print("Take-off thr.");
            break;
        default:
            lcd.print("Unknown error");
            break;
    }
}

