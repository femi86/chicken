/*
  Revised “Smart Door” Sketch
  --------------------------------
  - Removes most large blocking delays
  - Recomputes sunrise/sunset once per day
  - Eliminates the daily reset
  - Fixes certain bugs (e.g. == instead of =)
  - Uses non-blocking “millis()” approach for repeated tasks
*/

#include <RTClib.h>
#include <Wire.h>
#include <rgb_lcd.h>
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
RTC_DS1307 rtc;   // SDA (A4), SCL (A5)
rgb_lcd lcd;       // same lines as RTC (I2C bus)

#define REL_LAMP    2  // Relay for heating lamp
#define REL_BLINK   3  // Relay for blinking light
#define REL_OP      4  // Relay to open the door
#define REL_CL      5  // Relay to close the door
#define CL_SENS     6  // Closed-limit switch
#define ADJUST_TIME 7  // Pin used to adjust time (not heavily used in code)
#define REL_RD      8  // Relay for radio or motion sensor light
#define OP_SENS     9  // Open-limit switch
#define ADD_POWER   10 // Additional power pin
#define RESET_PIN   12 // (No longer used for daily resets!)

#define TEMP_SENS   A1 // Temperature sensor
#define LIGHT_SENS  A2 // Light sensor

// ---------------------------------------------------------------------------
// Operational Parameters
// ---------------------------------------------------------------------------
#define test_mode   0  // If >0, some debug / test behavior
#define sens        1  // Hysteresis threshold for sensors
#define criticalT   2  // Temp in °C below which lamp is turned on
#define DAWN        80 // Light sensor threshold for dawn
#define DAY         600// Light threshold for day
#define buff        120// Buffer (minutes) around sunrise/sunset

// Relay logic
#define inv_rel     1  // If 1, relay ON = LOW, OFF = HIGH
#define T_MOT       20 // Time for motor to fully open or close (seconds)
#define second_1    1000 // 1 second in ms

// Sunrise/sunset location/timezone
double lat = 47.3739;
double lon = 8.5451;
int    tz  = 2;       // base timezone offset
int    spring = 86;   // DOY for daylight saving start
int    fall   = 304;  // DOY for daylight saving end

// Sunrise/sunset results (in minutes from midnight)
int srise = 0;
int sset  = 0;

// Additional
int lampState    = 0;  // 0 = off, 1 = on
int v_on, v_off;       // Relay signals for ON/OFF
int day_ct       = 0;  // how many times we have read 'day' in a row
int count        = 0;  // used to count how many times we see “dawn”

// Number of sensor readings for averaging
int INTEG         = 15;   // default 15
int RPT           = 5;    // how many repeated “bright” checks for dawn
int average_time  = 500;  // ms delay between each sensor read
int waiting       = 2;    // minutes in certain states

// We’ll use these to manage repeated tasks in loop() via millis()
unsigned long lastSensorRead       = 0;     // last time sensors were read
unsigned long sensorReadInterval   = 5000;  // read sensors every 5s (example)

unsigned long lastStateCheck       = 0;     // last time we checked the day state
unsigned long stateCheckInterval   = 60000; // check state every 60s

unsigned long lastLCDUpdate        = 0;     // last time we updated LCD
unsigned long lcdUpdateInterval    = 5000;  // update LCD every 5s

// We’ll store sensor readings globally so all code can access them
int currentTemp     = 0;
int currentLightVal = 0;

// Keep track of current state
enum timeofday { night_, dawn_, day_, sundown_, dusk_, between_states_ };
timeofday currentState = between_states_;

// For daily sunrise/sunset updates
int lastCalculatedDay = -1; // store day of month or day of year last updated

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
    case 1:  break;                       // day_t is day of january
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
//   Returns values via global srise, sset (minutes from midnight)
// ---------------------------------------------------------------------------
void sun_time(double LAT, double LON, int &TZ, DateTime now) {
  int day_y = now.unixtime() / 86400L;
  int day_t = check_date(now);

  // Adjust for daylight saving if outside [spring..fall]
  int localTZ = TZ; // store original
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
  
  int noon = int(720 - 4*LON - eqtime + localTZ*60); // minutes from midnight
  double ha = degs(acos(cos(rads(90.833)) / (cos(rads(LAT)) * cos(rads(sd))) 
                  - tan(rads(LAT))*tan(rads(sd))));
  srise = noon - (int)(ha*4);
  sset  = noon + (int)(ha*4);
}

// ---------------------------------------------------------------------------
// Function: Average Temperature
//   (Non-blocking version would require rewriting this heavily; for now, keep shortish delays.)
// ---------------------------------------------------------------------------
int averageTemp(int integ, int avg_t) {
  long tmpSum = 0;
  for (int i = 0; i < integ; i++) {
    tmpSum += analogRead(TEMP_SENS);
    delay(avg_t); // small blocking
  }
  long avg = tmpSum / integ;
  
  // Convert to temperature (assuming Grove or similar thermistor)
  float R = 1023.0 / float(avg) - 1.0;
  R *= 100000.0;
  float tempK = 1.0 / (log(R / 100000.0) / 4299.0 + 1.0/298.15);
  float tempC = tempK - 273.15;
  return (int)round(tempC);
}

// ---------------------------------------------------------------------------
// Function: Average Light
//   (Simplified to remove the second big delay block. Adjust as needed.)
// ---------------------------------------------------------------------------
int averageLight(int integ, int avg_t) {
  long sum = 0;
  for (int i = 0; i < integ; i++) {
    sum += analogRead(LIGHT_SENS);
    delay(avg_t);
  }
  return (sum / integ);
}

// ---------------------------------------------------------------------------
// Turn Lamp On/Off based on temperature
// ---------------------------------------------------------------------------
void TempLamp() {
  if (currentTemp <= criticalT) {
    digitalWrite(REL_LAMP, v_on);
    lampState = 1;
  } else {
    digitalWrite(REL_LAMP, v_off);
    lampState = 0;
  }
}

// ---------------------------------------------------------------------------
// Door Control
//   state can be "open" or "close"
//   This is a short blocking approach, but door movement is often short in real life
// ---------------------------------------------------------------------------
void Door(const char* state){
  int SENS, REL, FREE, END;
  
  if (strcmp(state, "close") == 0) {
    SENS = CL_SENS;
    REL  = REL_CL;
    FREE = 1;
    END  = 0;
  } else { // "open"
    SENS = OP_SENS;
    REL  = REL_OP;
    FREE = 0;
    END  = 1;
  }
  
  if (digitalRead(SENS) == FREE) {
    // Keep the relay on until the door limit switch indicates it’s done
    while (digitalRead(SENS) != END) {
      digitalWrite(REL, v_on);
      delay(50); // short block
    }
    // Turn off motor
    digitalWrite(REL, v_off);
  }
}

// ---------------------------------------------------------------------------
// Determine if it is night/dawn/day_/sundown/dusk or between states
// ---------------------------------------------------------------------------
timeofday checktime() {
  DateTime now = rtc.now();
  int tm_mins = now.hour() * 60 + now.minute();

  // If time is well after sunset + buffer or well before sunrise - buffer → night
  if ((tm_mins > (sset + buff)) || (tm_mins < (srise - buff))) {
    day_ct = 0;
    return night_;
  }
  // If within the sunrise +/- buff window → dawn or night
  else if (tm_mins >= (srise - buff) && tm_mins <= (srise + buff)) {
    if (currentLightVal > DAWN) {
      if (count == RPT) {
        count = RPT + 1;
      } else {
        count++;
      }
      day_ct = 0;
      return dawn_;
    } else {
      count = 0;
      day_ct = 0;
      return night_;
    }
  }
  // If between sunrise+buff and sunset-buff → day
  else if (tm_mins > (srise + buff) && tm_mins < (sset - buff)) {
    day_ct++;
    return day_;
  }
  // If near sunset → sundown/dusk
  else if (tm_mins >= (sset - buff)) {
    if (currentLightVal > DAWN) {
      day_ct = 0;
      return sundown_;
    } else {
      if (digitalRead(CL_SENS) == 1) {
        day_ct = 0;
        return dusk_;
      } else {
        day_ct = 0;
        return night_;
      }
    }
  }
  // fallback
  return between_states_;
}

// ---------------------------------------------------------------------------
// Simple LED blink on board LED (non-blocking version possible, but short is okay).
// ---------------------------------------------------------------------------
void ledBlink(int SEC_ON, int SEC_OFF){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(SEC_ON);
  digitalWrite(LED_BUILTIN, LOW);
  delay(SEC_OFF);
}

// ---------------------------------------------------------------------------
// For a “night blink” effect (e.g., strobe)
/// --------------------------------------------------------------------------
void nightBlink(int durationSec){
  digitalWrite(REL_BLINK, v_on);
  unsigned long start = millis();
  while ((millis() - start) < (unsigned long)durationSec * 1000UL) {
    // strobe for 'durationSec' seconds
    // we could do something more elaborate here
  }
  digitalWrite(REL_BLINK, v_off);
}

// ---------------------------------------------------------------------------
// Return door state: 0 = closed, 1 = open
// ---------------------------------------------------------------------------
int doorState(){
  // If closed sensor is 0, open sensor is 0 => door_status=0
  // If closed sensor is 1, open sensor is 1 => door_status=1
  if (digitalRead(CL_SENS) == 0 && digitalRead(OP_SENS) == 0) {
    return 0; // fully closed
  }
  else if (digitalRead(CL_SENS) == 1 && digitalRead(OP_SENS) == 1) {
    return 1; // fully open
  }
  // Could add partial states if you want
  return -1; // unknown
}

// ---------------------------------------------------------------------------
// Print times and relevant info to LCD and Serial
// ---------------------------------------------------------------------------
void printTimes(int r, int g, int b, DateTime now) {
  int door_status = doorState();
  lcd.setRGB(r, g, b);
  lcd.clear();
  lcd.setCursor(0, 0);

  // Buffers to hold text
  char buff1[16];
  char buff2[16];
  
  snprintf(buff1, sizeof(buff1),
           "%02d:%02d L%03dD%dT%02d",
           now.hour(), now.minute(), currentLightVal, door_status, currentTemp);
  snprintf(buff2, sizeof(buff2),
           "SR%02d:%02dSD%02d:%02d",
           srise / 60, srise % 60, sset / 60, sset % 60);

  lcd.print(buff1);
  lcd.setCursor(0,1);
  lcd.print(buff2);

  // Also print to Serial
  Serial.print(F("Time ")); Serial.print(now.hour()); 
  Serial.print(F(":")); Serial.print(now.minute());
  Serial.print(F(", Sunrise ")); Serial.print(srise/60); 
  Serial.print(F(":")); Serial.print(srise%60);
  Serial.print(F(" Sunset ")); Serial.print(sset/60); 
  Serial.print(F(":")); Serial.print(sset%60);
  Serial.print(F(", Light=")); Serial.print(currentLightVal);
  Serial.print(F(", Door=")); Serial.print(door_status);
  Serial.print(F(", Temp=")); Serial.print(currentTemp);
  Serial.print(F(", Lamp=")); Serial.println(lampState);
}

// ---------------------------------------------------------------------------
// Recompute sunrise/sunset once per day (midnight check)
// ---------------------------------------------------------------------------
void dailySunTimeUpdate() {
  DateTime now = rtc.now();
  int hour   = now.hour();
  int minute = now.minute();

  // If we are at midnight (00:00), recalc sun times if not done yet
  if (hour == 0 && minute == 0) {
    int dayOfYear = check_date(now);
    if (dayOfYear != lastCalculatedDay) {
      sun_time(lat, lon, tz, now);
      lastCalculatedDay = dayOfYear;
      Serial.println(F("Sunrise/Sunset updated for new day."));
    }
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); // keep it high, no daily reset

  pinMode(ADD_POWER, OUTPUT);
  digitalWrite(ADD_POWER, HIGH);

  pinMode(ADJUST_TIME, OUTPUT);
  digitalWrite(ADJUST_TIME, HIGH);

  lcd.begin(16,2);

  // RTC init
  if (!rtc.begin()) {
    while(true) {
      ledBlink(50, 50); // Blink to show error
    }
  }
  if (! rtc.isrunning()) {
    ledBlink(500,100);
    // set RTC to compile time if needed:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Relay pinModes
  pinMode(REL_OP,    OUTPUT);
  pinMode(REL_CL,    OUTPUT);
  pinMode(REL_RD,    OUTPUT);
  pinMode(REL_LAMP,  OUTPUT);
  pinMode(REL_BLINK, OUTPUT);

  pinMode(CL_SENS,   INPUT_PULLUP);
  pinMode(OP_SENS,   INPUT);

  // If inv_rel == 1, ON=LOW, OFF=HIGH
  if (inv_rel == 1) {
    v_on  = LOW;
    v_off = HIGH;
  } else {
    v_on  = HIGH;
    v_off = LOW;
  }

  // Set all relays OFF
  digitalWrite(REL_OP,    v_off);
  digitalWrite(REL_CL,    v_off);
  digitalWrite(REL_LAMP,  v_off);
  digitalWrite(REL_RD,    v_off);
  digitalWrite(REL_BLINK, v_off);

  // First-time sunrise/sunset calculation
  DateTime now = rtc.now();
  sun_time(lat, lon, tz, now);
  lastCalculatedDay = check_date(now);

  // Do an initial sensor read
  currentTemp = averageTemp(5, 100);
  currentLightVal = averageLight(5, 50);
}

// ---------------------------------------------------------------------------
// Main Loop (Non-blocking approach)
// ---------------------------------------------------------------------------
void loop() {
  unsigned long currentMillis = millis();
  DateTime now = rtc.now();

  // 1) Recompute sunrise/sunset once a day
  dailySunTimeUpdate();

  // 2) Periodically read sensors
  if (currentMillis - lastSensorRead >= sensorReadInterval) {
    lastSensorRead = currentMillis;
    currentTemp     = averageTemp(5, 100);
    currentLightVal = averageLight(5, 50);
    TempLamp(); // Check lamp right after reading temp
  }

  // 3) Periodically check which state (night/dawn/day/sundown/dusk/between)
  if (currentMillis - lastStateCheck >= stateCheckInterval) {
    lastStateCheck = currentMillis;
    currentState   = checktime(); // updates global “currentState”
    
    // React to the state
    switch (currentState) {
      case night_:
        Serial.println(F("Arduino mode: Night"));
        // close door if needed
        Door("close");
        // Turn off radio
        digitalWrite(REL_RD, v_off);
        // Optionally blink
        nightBlink(2); 
        break;
        
      case dawn_:
        Serial.println(F("Arduino mode: Dawn"));
        if (count == RPT) {
          Door("open");
        } else if (count == RPT + 1) {
          Door("open");
        } 
        // else wait for more dawn confirmations
        break;
        
      case day_:
        Serial.println(F("Arduino mode: Day"));
        // ensure door is open
        Door("open");
        // radio off
        digitalWrite(REL_RD, v_off);
        break;
        
      case sundown_:
        Serial.println(F("Arduino mode: Sundown"));
        // if door is still open, turn on radio? (As per original logic)
        if (digitalRead(CL_SENS) == 1) {
          digitalWrite(REL_RD, v_on);
        } else {
          digitalWrite(REL_RD, v_off);
        }
        // blink the relay
        digitalWrite(REL_BLINK, v_on);
        break;
        
      case dusk_:
        Serial.println(F("Arduino mode: Dusk"));
        digitalWrite(REL_BLINK, v_on);
        // close door
        Door("close");
        digitalWrite(REL_RD, v_off);
        // We do NOT reset the Arduino anymore
        digitalWrite(RESET_PIN, HIGH);
        break;
        
      case between_states_:
        Serial.println(F("Arduino mode: Between States"));
        break;
    }
  }

  // 4) Update LCD every so often
  if (currentMillis - lastLCDUpdate >= lcdUpdateInterval) {
    lastLCDUpdate = currentMillis;
    printTimes(0, 255, 0, now); // green color, or choose color based on state
  }

  // 5) (Optional) Additional tasks, if needed
  // ...
}

