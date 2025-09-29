/*
  Consolidated "Smart Door" Sketch with RTC Stuck Check
  -----------------------------------------------------
  - Polls RTC and sensors once per minute (blocking)
  - Median filter on temperature
  - Zero-padded time for RTC
  - Sunrise/sunset daily update
  - Two Serial lines per cycle:
       1) Sensor values
       2) RTC time, sunrise, sunset
  - Checks if RTC time has not advanced => attempt re-init + forced reset
*/

#include <Wire.h>
#include <RTClib.h>
#include <rgb_lcd.h>
#include <math.h>
#include <stdio.h>
// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
RTC_DS3231 rtc;   // SDA (A4), SCL (A5)
rgb_lcd lcd;       // same lines as RTC (I2C bus)

#define REL_LAMP    2   // Relay for heating lamp
#define REL_BLINK   3   // Relay for blinking light
#define REL_OP      4   // Relay to open the door
#define REL_CL      5   // Relay to close the door
#define CL_SENS     6   // Closed-limit switch
#define ADJUST_TIME 7   // Pin used to adjust time
#define REL_RD      8   // Relay for radio or motion sensor light
#define OP_SENS     9   // Open-limit switch
#define RESET_PIN   12  // If pulled LOW => hardware reset (wired to Arduino RST)

#define TEMP_SENS   A1  // Temperature sensor analog pin
#define LIGHT_SENS  A2  // Light sensor analog pin

// ---------------------------------------------------------------------------
// Operational Parameters
// ---------------------------------------------------------------------------
#define test_mode   0   // If >0, some debug / test behavior
#define sens        1   // Hysteresis threshold for sensors (unused here)
#define criticalT   4   // Temp in °C below which lamp is turned on
#define DAWN        80  // Light sensor threshold for dawn
#define DAY         600 // Light threshold for day
#define buff        90 // Buffer (minutes) around sunrise/sunset

// Relay logic
#define inv_rel     1   // If 1, relay ON = LOW, OFF = HIGH
#define T_MOT       20  // Time for motor to fully open/close (seconds)
#define second_1    1000// 1 second in ms
#define RPT         5 // the repetition for dawn


// Sunrise/sunset location/timezone
double lat = 47.3739;
double lon = 8.5451;
int    tz  = 2;       // base timezone offset
int    spring = 86;   // DOY for daylight saving start
int    fall   = 304;  // DOY for daylight saving end

// Sunrise/sunset (minutes from midnight)
int srise = 0;
int sset  = 0;

// Additional
int lampState    = 0;   // 0 = off, 1 = on
int v_on, v_off;        // Relay signals for ON/OFF
int day_ct       = 0;   // times we have read 'day' in a row
int count        = 0;   // how many times we see “dawn”
float t;
int limitReached = -1;

// We'll keep final computed temp & light globally
int currentLightVal = 0;

// day-part enum
enum timeofday { night_, dawn_, day_, sundown_, dusk_, between_states_ };
timeofday currentState = between_states_;

// For daily sunrise/sunset updates
int lastCalculatedDay = -1; // store the day-of-year last updated

// ---------------------------------------------------------------------------
// For "RTC stuck" detection
// ---------------------------------------------------------------------------
uint32_t lastValidUnix = 0; // store last successful unixtime

// for serial command handling
#define MAX_COMMAND_LENGTH 10
// for serial command handling
#define MAX_COMMAND_LENGTH 10
char inputBuffer[MAX_COMMAND_LENGTH];
int inputPos = 0;

// ---------------------------------------------------------------------------
// Utility: Return the day of year
// ---------------------------------------------------------------------------
int check_date(DateTime now) {
  int day_t = now.day();
  int feb = 28;
  int year = now.year();

  // leap year check
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    feb = 29;
  }

  switch (now.month()) {
    case 1:  break;                       
    case 2:  day_t += 31;                break;
    case 3:  day_t += 31 + feb;          break;
    case 4:  day_t += 31 + feb + 31;     break;
    case 5:  day_t += 31 + feb + 31 + 30; break;
    case 6:  day_t += 31 + feb + 31 + 30 + 31; break;
    case 7:  day_t += 31 + feb + 31 + 30 + 31 + 30; break;
    case 8:  day_t += 31 + feb + 31 + 30 + 31 + 30 + 31; break;
    case 9:  day_t += 31 + feb + 31 + 30 + 31 + 30 + 31 + 31; break;
    case 10: day_t += 31 + feb + 31 + 30 + 31 + 30 + 31 + 31 + 30; break;
    case 11: day_t += 31 + feb + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31; break;
    case 12: day_t += 31 + feb + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30; break;
  }
  return day_t;
}

// ---------------------------------------------------------------------------
// Sunrise/Sunset Calculation
// ---------------------------------------------------------------------------
void sun_time(double LAT, double LON, int &TZ, DateTime now) {
  int day_y = now.unixtime() / 86400L;  // days since 1970
  int day_t = check_date(now);          // day of year

  // Adjust for daylight saving if outside [spring..fall]
  int localTZ = TZ; 
  if (day_t < spring || day_t > fall) {
    localTZ = TZ - 1;
  }

  double j_day = double(day_y) + 25569.0 + 2415018.5
                 + (now.hour() / 24.0 + now.minute() / 1440.0)
                 - double(localTZ) / 24.0;
  double j_cen = (j_day - 2451545) / 36525.0;
  double moe = 23 + (26 + ((21.448 - j_cen * (46.815 + j_cen * (0.00059 -
                    j_cen * 0.001813)))) / 60.0) / 60.0;
  
  double gmas = 357.52911 + j_cen * (35999.05029 - (0.0001537 * j_cen));

  auto rads = [](double x){ return x * 3.14159265358979 / 180.0; };
  auto degs = [](double x){ return x * 180.0 / 3.14159265358979; };
  
  double sec = sin(rads(gmas)) * (1.914602 - j_cen*(0.004817 + 0.000014*j_cen))
               + sin(rads(2*gmas))*(0.019993 - 0.000101*j_cen)
               + sin(rads(3*gmas))*0.000289;
  double gml = fmod(280.46646 + j_cen*(36000.76983 + 0.0003032*j_cen), 360.0);
  double stl = gml + sec;
  double oc  = moe + 0.00256*cos(rads(125.04 - 1934.136*j_cen));
  double sal = stl - 0.00569 - 0.00478*sin(rads(125.04 - 1934.136*j_cen));
  double sd  = degs(asin(sin(rads(oc)) * sin(rads(sal))));
  double eeo = 0.016708634 - j_cen*(0.000042037 + 0.0000001267*j_cen);
  double y   = tan(rads(oc/2.0))*tan(rads(oc/2.0));
  double eqtime = 4.0 * degs(y*sin(2.0*rads(gml))
                   - 2.0*eeo*sin(rads(gmas))
                   + 4.0*eeo*y*sin(rads(gmas))*cos(2.0*rads(gml))
                   - 0.5*y*y*sin(4.0*rads(gml))
                   - 1.25*eeo*eeo*sin(2.0*rads(gmas)));
  
  int noon = int(720 - 4*LON - eqtime + localTZ*60);
  double ha = degs(acos(cos(rads(90.833)) / (cos(rads(LAT)) * cos(rads(sd))) 
                  - tan(rads(LAT))*tan(rads(sd))));
  srise = noon - (int)(ha*4);
  sset  = noon + (int)(ha*4);
}

// Read Light (simple averaging)
// ---------------------------------------------------------------------------
int readLightSensor(int samples, int delayMs) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(LIGHT_SENS);
    delay(delayMs);
  }
  return sum / samples;
}

// ---------------------------------------------------------------------------
// Turn Lamp On/Off
// ---------------------------------------------------------------------------
void TempLamp() {
  if (t <= criticalT) {
    digitalWrite(REL_LAMP, v_on);
    lampState = 1;
  } else {
    digitalWrite(REL_LAMP, v_off);
    lampState = 0;
  }
}

// ---------------------------------------------------------------------------
// Door Control
// ---------------------------------------------------------------------------
void Door(const char* state){
  int SENS, REL, LIM;
  int consRead = 0;
  int FREE = 0;
  int END = 1;
  if (state=="close") {
    SENS = CL_SENS;
    REL  = REL_CL;
    LIM = 0;
  } 
  else if (state=="open"){
    SENS = OP_SENS;
    REL  = REL_OP;
    LIM = 1;
  }
  digitalWrite(REL, v_on);
    while (true) {
      digitalWrite(REL, v_on);
      delay(50);
      if (digitalRead(SENS) == END){
        consRead++;
      }else{
	consRead = 0;
      }
      if (consRead >= 3){
	limitReached = LIM;
	break;
	}
    }
  digitalWrite(REL, v_off);
}

// ---------------------------------------------------------------------------
// Return door state: 0 = closed, 1 = open, -1 = unknown
// ---------------------------------------------------------------------------
int doorState() {
  if (limitReached == 0) {
    return 0; // closed
  } 
  else if (limitReached == 1) {
    return 1; // open
  }
  return -1;  // partial or error
}

// ---------------------------------------------------------------------------
// Determine time of day
// ---------------------------------------------------------------------------
enum timeofday checktime(DateTime now) {
  int tm_mins = now.hour()*60 + now.minute();

  if ((tm_mins > (sset + (buff))) || (tm_mins < (srise - buff))) {
    day_ct = 0;
    count = 0;
    return night_;
  }
  else if (tm_mins >= (srise - buff) && tm_mins <= (srise + buff)) {
    if (currentLightVal > DAWN) {
      if (count < RPT) {
        count ++;
      }
      day_ct = 0;
      return dawn_;
    } else {
      count = 0;
      day_ct = 0;
      return night_;
    }
  }
  else if (tm_mins > (srise + buff) && tm_mins < (sset - buff)) {
    day_ct++;
    count = 0;
    return day_;
  }
  else if (tm_mins >= (sset - buff)) {
    if (currentLightVal > DAWN) {
      day_ct = 0;
      return sundown_;
    } else {
      if (limitReached == 1) {
        day_ct = 0;
        return dusk_;
      } else {
        day_ct = 0;
        return night_;
      }
    }
  }
  return between_states_;
}

// ---------------------------------------------------------------------------
// Blink the built-in LED
// ---------------------------------------------------------------------------
void ledBlink(int onMs, int offMs) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(onMs);
  digitalWrite(LED_BUILTIN, LOW);
  delay(offMs);
}

// ---------------------------------------------------------------------------
// Night blink on external relay
// ---------------------------------------------------------------------------
void nightBlink(int durationSec) {
  digitalWrite(REL_BLINK, v_on);
  delay(durationSec * 1000UL);
  digitalWrite(REL_BLINK, v_off);
}

// ---------------------------------------------------------------------------
// Print sensor values on one line, and time info on the next line
// Also displays time info on the LCD
// ---------------------------------------------------------------------------
void printTimes(DateTime now) {
  int door_status = doorState();
  // --- LCD update (2 lines) ---
  lcd.setRGB(0, 255, 0); 
  lcd.clear();
  lcd.setCursor(0, 0);

  // Buffers for LCD lines
  char buffTop[16];
  char buffBot[16];

  // Zero-padded HH:MM
  snprintf(buffTop, sizeof(buffTop),
           "%02d:%02d L%03dD%dT%02d",
           now.hour(), now.minute(), currentLightVal, door_status, t);

  snprintf(buffBot, sizeof(buffBot),
           "SR%02d:%02dSD%02d:%02d",
           srise/60, srise%60, sset/60, sset%60);

  // Show on LCD
  lcd.print(buffTop);
  lcd.setCursor(0,1);
  lcd.print(buffBot);

  // --- SERIAL OUTPUT: 2 lines ---
  // 1) Sensor line
  Serial.print("Temp=");
  Serial.print(t);     // e.g. 22
  Serial.print("C, Light=");
  Serial.print(currentLightVal); // e.g. 300
  Serial.print(", Door=");
  Serial.print(door_status);     // e.g. 1 or 0
  Serial.print(", Lamp=");
  Serial.println(!digitalRead(REL_LAMP));     // e.g. 1 or 0, normally lampState but bypassed currently
  delay(10);

  // 2) Time line
  char timeStr[6];
  char srStr[6];
  char ssStr[6];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d", now.hour(), now.minute());
  snprintf(srStr,   sizeof(srStr),   "%02d:%02d", srise/60, srise%60);
  snprintf(ssStr,   sizeof(ssStr),   "%02d:%02d", sset/60,  sset%60);

  Serial.print("Time=");
  Serial.print(timeStr);
  Serial.print(", Sunrise=");
  Serial.print(srStr);
  Serial.print(", Sunset=");
  Serial.println(ssStr);

  // Optional short delay to ensure data fully transmits
  delay(10);
}

// ---------------------------------------------------------------------------
// Recompute sunrise/sunset once a day at midnight
// ---------------------------------------------------------------------------
void dailySunTimeUpdate(DateTime now) {
  int hour   = now.hour();
  int minute = now.minute();
  int dayOfYear = check_date(now);

  if (hour == 0 && minute == 0 && (dayOfYear != lastCalculatedDay)) {
    sun_time(lat, lon, tz, now);
    lastCalculatedDay = dayOfYear;
    Serial.println(F("Sunrise/Sunset updated for new day."));
  }
}

// ---------------------------------------------------------------------------
// Check if RTC got stuck: compare new unixtime to the last one
// If stuck => re-init I2C + forcibly reset
// ---------------------------------------------------------------------------
void checkRTCStuck(uint32_t newUnix) {
  if (newUnix == lastValidUnix) {
    Serial.println(F("WARNING: RTC time not advancing! Attempting re-init and forced reset..."));

    // Re-init I2C / RTC
    Wire.end();
    Wire.begin();
    rtc.begin();

    // Force a hardware reset:
    digitalWrite(RESET_PIN, LOW); 
    // The code won't return from here on many boards, as it triggers a reset
  } else {
    lastValidUnix = newUnix;
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (inputPos > 0) {
        inputBuffer[inputPos] = '\0';
	
	if (strcmp(inputBuffer,"open") == 0) {
	  Serial.println(F("Manual Command Received: Open Door"));
	  Door("open");
	}
	else if (strcmp(inputBuffer, "close") == 0) {
	  Serial.println(F("Manual Command received: Close Door"));
	  Door("close");
	}
	else {
	  Serial.print(F("Unknown Command: "));
	  Serial.println(inputBuffer);
	}
	inputPos = 0;
      }
    }
    else {
      if (inputPos < (MAX_COMMAND_LENGTH - 1)) {
        inputBuffer[inputPos++] = inChar;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); // keep it high (no daily reset)

  pinMode(ADJUST_TIME, OUTPUT);
  digitalWrite(ADJUST_TIME, HIGH);

  lcd.begin(16,2);
  // Initialize RTC + I2C
  Wire.begin();
  if (!rtc.begin()) {
    while(true) {
      ledBlink(50, 50); // error blink
    }
  }
  if (!rtc.lostPower()) {
    // If desired, set the RTC:
    ledBlink(500, 100);
  }
  //rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  

  // Relay pinModes
  pinMode(REL_OP,    OUTPUT);
  pinMode(REL_CL,    OUTPUT);
  pinMode(REL_RD,    OUTPUT);
  pinMode(REL_LAMP,  OUTPUT);
  pinMode(REL_BLINK, OUTPUT);

  pinMode(CL_SENS,   INPUT_PULLUP);
  pinMode(OP_SENS,   INPUT_PULLUP);

  if (inv_rel == 1) {
    v_on  = LOW;
    v_off = HIGH;
  } else {
    v_on  = HIGH;
    v_off = LOW;
  }

  // Set all relays off
  digitalWrite(REL_OP,    v_off);
  digitalWrite(REL_CL,    v_off);
  digitalWrite(REL_LAMP,  v_off);
  digitalWrite(REL_RD,    v_off);
  digitalWrite(REL_BLINK, v_off);

  // Initial sunrise/sunset
  DateTime initNow = rtc.now();
  sun_time(lat, lon, tz, initNow);
  lastCalculatedDay = check_date(initNow);


  // Store the initial valid time
  lastValidUnix = initNow.unixtime();
  delay(2000);
}

// ---------------------------------------------------------------------------
// Main Loop: Once-per-minute blocking approach
// ---------------------------------------------------------------------------
void loop() {
  
  delay(100);// 1) read current time
  for (int i; i < 20; i++)  {
    //digitalWrite(REL_OP,v_on);
    delay(500);
  }
  delay(10000);
  }	
