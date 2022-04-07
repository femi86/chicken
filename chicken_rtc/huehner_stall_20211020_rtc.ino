#include "RTClib.h"
#include "math.h"
#include "Wire.h"

RTC_DS1307 rtc;// SDA is connecteed to A4, SCL to A5, power to A3, gnd to gnd

#define REL_OP 4 //def 4 this is the command to open the door, on relay 3
#define REL_CL 5 // def 5 this is the command to close the door, on relay 2
#define REL_RD 8 // def 8 this is the switch to turn on the radio and the motion sensor light, on relay 4
#define REL_LAMP 2 // def 2 this is the switch to turn on the heating lamp, on relay 1
#define REL_BLINK 3 // the relay for the blinker
#define CL_SENS 6 // def 6 the switch, that bridges ground to digital pin 6 to detect when it is closed (0 is closed, 1 is open)
#define LIGHT_SENSOR A2 // def a2
#define TEMP_SENS A1 // def a1
#define resetPin 12 // reset pin for manual reset
#define test_mode 0 // if in test mode, value is greater than 0

// functon params
#define sens 1 // def 1, this is the sensitivity for the hysteresis evaluation, which is the "noise" on the sensor, it has to be determined manually depending on the position of the sensor. try it a few times with the sensor_det code
#define criticalT 2 // def 2, the critical temperature (in °C) below which the lamp is turned on
#define DAWN 50 // this is the value at which the door starts opening (it is similar to dusk)
#define DAY 600 // def 700, this is the value at which it is definitely day
#define buff 90 // this is how long the dawn should last after sunrise, in minutes

// relay params
#define inv_rel 1 // def 1, if the relay is turned on with ground, otherwise 0
#define T_MOT 40 // def 27, the time it takes the motor to close in seconds
#define second_1 1000 // def 1000, one second, can be shortened for debug 

// location params

double lat {47.3739}; // your latitude
double lon  {8.5451}; // your longitude
int tz {1}; // your timezone vs UTC, with winter time 2021. change to 2 in january 2022
int spring {86}; // the day number when daylight saving comes in this year (2022)
int fall {304}; // the day number when daylight saving goes away this year (2022)

// script speed params
int INTEG {15}; // def 15, how many iterations for the sensor averaging
const int RPT {15}; // def 15, how many measurements before deciding it is indeed dawn (to exit the first loop)
int average_time {500}; // def 500, this defines how fast the sensor integrates, in ms
int waiting {2}; // time to wait within a state

// init the variables used in  script
int count {0};
int day_ct {0};
int v_on;
int v_off;
const double rad {57296}; // denominator for radians
const double deg {57296}; // numerator for degree
int srise {0};
int sset {0};

// function definition, mathematical
int sun_time(double LAT, double LON, int TZ, DateTime now){
    int day_y = now.unixtime() / 86400L;
    if (day_y < spring && now.year() == 2022 || day_y > fall && now.year() == 2022){
      // TZ -= 1; // change to winter time. uncomment in january 2022
    }
    double j_day = day_y + 25569 + 2415018.5 + (int(now.hour())/24 - TZ/24); // the 25569 is the time in days between excel's 0 and unix 
    double j_cen = (j_day - 2451545)/36525;
    double moe = 23 + (26 + ((double(21.448) - j_cen * (double(46.815) + j_cen * (double(0.00059) - 
                            (j_cen*double(0.001813))))))/float(60))/float(60);
    double gmas = double(357.52911) + j_cen * (double(35999.05029) - (double(0.0001537) * j_cen));
    double sec = sin(float(gmas * 1000/rad))*(double(1.914602) - j_cen * (double(0.004817) + (double(0.000014) * j_cen))) + 
                  sin(double(2*gmas * 1000/rad))*(double(0.019993) - (double(0.000101)*j_cen)) + 
                  sin(double(3*gmas*1000/rad))*double(0.000289);
    double gml = fmod(double(280.46646) + j_cen * (double(36000.76983) + (j_cen * double(0.0003032))),float(360.00));
    double stl = gml + sec;
    double oc = moe + double(0.00256) * cos((float(125.04)-double(1934.136)*j_cen)*1000/rad); // degrees
    double sal = stl - double(0.00569)-(double(0.00478)*sin((float(125.04) -(double(1934.136) * j_cen))*1000/rad)); // degrees
    double sd = deg/1000*asin(sin(double(oc * 1000/rad))*sin(double(sal * 1000/rad))); // degrees
    
    double eeo = double(0.016708634) - (j_cen * (double(0.000042037) + (double(0.0000001267) * j_cen)));
    double y = tan(double(oc/2.0*long(1000)/rad))*tan(double(oc/2.0*1000/rad));
    double eqtime = 4.0 * deg/1000 * (y * sin(double(2.0*gml*long(1000)/rad)) - 
                                        (2.0*eeo*sin(double(gmas*1000/rad))) + 
                                        (4.0*eeo*y*sin(double(gmas*1000/rad)) * cos(double(2.0*gml*1000/rad))) -
                                        (float(0.5)*y*y*sin(double(4.0*1000/rad*gml))) -
                                        (float(1.25)*eeo*eeo*sin(double(2.0*gmas*1000/rad)))); // minutes
    int noon = 720 - 4*LON - eqtime + TZ*60; // minutes
    double ha = acos((cos(double(90.833) *1000/rad)/(cos(LAT*1000/rad) * cos(sd*1000/rad)))- (tan(LAT*1000/rad) * tan(sd*1000/rad)))*deg/1000; // degrees
    srise = noon - ha * 4;
    sset = noon + ha * 4;
    }


int averageTemp(){
  int tmp = 0;
  for (int i = 1; i<=INTEG; i++){
    tmp += analogRead(TEMP_SENS);
    delay(average_time);
  }
  float R = 1023.0/(tmp/INTEG)-1.0;
  R = 100000*R;
  float temp = 1.0/(log(R/100000)/4299+1/298.15)-273.15;
  return temp;
}

int averageLight(){
  int result = 0;
  for (int i = 1; i <= INTEG; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(average_time); 
  }
  delay(second_1 * 5);
  for (int i = 1; i <= INTEG; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(average_time); 
  }
  return result /= (INTEG*2);
}

// operational functions

int TempLamp(){
  if (averageTemp() <= criticalT){
    digitalWrite(REL_LAMP,v_on);
    delay(second_1);
  }
  else {
    digitalWrite(REL_LAMP,v_off);
    delay(second_1);
  }
}

enum timeofday{night, dawn, day_, sundown, dusk, between_states};

int checktime(){
  DateTime now = rtc.now();
  int light_val = averageLight();
  int tm_mins = (now.hour()*60)+now.minute();
  
  if ((tm_mins > (sset+buff)) || (tm_mins < (srise - buff))){
    Serial.println("night code");
    day_ct = 0;
    return night;
  }
  else if ((tm_mins >= (srise - buff)) && (tm_mins <= (srise + buff))){
    if (light_val > DAWN){
      Serial.println("Dawn code");
      count += 1;
      day_ct = 0;
      return dawn;
    }
    else {
      Serial.println("Dawn code, night case");
      count = 0;
      day_ct = 0;
      return night;
    }
  }
  else if ((tm_mins > (srise + buff)) && (tm_mins < (sset - buff))){
    Serial.println("Day code");
    day_ct += 1;
    return day_;
  }
  else if (tm_mins >= (sset - buff)){
    if (light_val > DAWN){
      Serial.println("sundown code");
      day_ct = 0;
      return sundown;
    }
    else {
      if (digitalRead(CL_SENS) > 0){
        Serial.println("dusk code");
        day_ct = 0;
        return dusk;
      }
      else {
        Serial.println("dusk code, night case");
        return night;
      }
    }
  }
  else {
    Serial.println("between cases");
    return between_states;
  }
}

int openDoor(){
  digitalWrite(REL_OP,v_on);
  for (int i = 0; i<T_MOT; i++){
    delay(second_1);
  }
  digitalWrite(REL_OP,v_off);
  delay(second_1/10);
}

int closeDoor(){
  digitalWrite(REL_CL,v_on);
  int secs = 0;
  while (true){
      delay(500);
      secs +=1;
      if (digitalRead(CL_SENS) < 1 || secs > T_MOT*2 ){// check if door closed, or if too long
        delay(500);
        digitalWrite(REL_RD,v_off);
        break;
    }
  }
  digitalWrite(REL_CL,v_off); // this is the automatic run
  delay(second_1);
}

int wait_minutes(int mins){
  if (mins == 0){
    delay(second_1);
  }
  else{
    for (int n = 0; n < mins; n++){
      for (int i = 0; i<60; i++){
        delay(second_1);
      }
    }
  }
}


int ledBlink(int SEC_ON, int SEC_OFF){
  for (int i = 0; i <= 5; i++){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(SEC_ON);
    digitalWrite(LED_BUILTIN,LOW);
    delay(SEC_OFF);
  }
}

int nightBlink(int duration){
  digitalWrite(REL_BLINK,v_on);
  for (int i = 0; i < duration; i++){
    delay(second_1);
  }
  digitalWrite(REL_BLINK,v_off);
  delay(second_1*5);
}

void(* resetFunc) (void) = 0; //declare reset function at address 0, to reset the arduino at the end of one loop (night)

void setup() {
  // put your setup code here, to run once:
  digitalWrite(resetPin,HIGH);
  pinMode(resetPin,OUTPUT);
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  digitalWrite(10,HIGH);
  if (! rtc.begin()) {
    while(true){
      ledBlink(50,50);
    }
  }
  if (! rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(REL_OP,OUTPUT);
  pinMode(REL_CL,OUTPUT);
  pinMode(REL_RD,OUTPUT);
  pinMode(REL_LAMP,OUTPUT);
  pinMode(REL_BLINK,OUTPUT);
  pinMode(CL_SENS,INPUT_PULLUP);
  if (inv_rel == 1){
    v_on = LOW;
    v_off = HIGH;
  }
  else{
    v_on = HIGH;
    v_off = LOW;
  }
  digitalWrite(REL_OP,v_off);
  digitalWrite(REL_CL,v_off);
  digitalWrite(REL_LAMP,v_off);
  digitalWrite(REL_RD,v_off);
  digitalWrite(REL_BLINK,v_off);
  DateTime now = rtc.now();
  sun_time(lat,lon,tz,now);
}
void loop() {
  // int analogValue = averageLight();
  int check = checktime();
  if (test_mode > 0){
    int INTEG = 1; // def 10, how many iterations for the sensor averaging
    const int RPT = 2; // def 15, how many measurements before deciding it is indeed dawn (to exit the first loop)
    int average_time = 50;
    int average_hyst = 1;
    int waiting = 0;
  }
  switch(check){
    case night:
      Serial.println("night executions");
      ledBlink(300,1500);
      TempLamp();
      nightBlink(80);
      digitalWrite(REL_RD, v_off);
      if (digitalRead(CL_SENS) > 0){
        closeDoor();
      }
      break;
    case dawn:
    Serial.println("dawn executions");
      ledBlink(300,300);
      nightBlink(10);
      digitalWrite(REL_RD, v_off);
      switch(count){
        case RPT:
          openDoor(); // if it is closed, open for 40 seconds
          digitalWrite(REL_LAMP,v_off);  
          break;
        default:
          delay(second_1*5);
          break;
        }
      break;
    case day_:
      Serial.println("day executions");
      ledBlink(1500,1500);
      if (digitalRead(CL_SENS) < 1 && day_ct == 1){
        openDoor();
      }
      wait_minutes(waiting);
      digitalWrite(REL_RD,v_off);
      break;
    case sundown:
      Serial.println("sundown executions");
      ledBlink(600,300);
      if (digitalRead(CL_SENS) > 0){
        digitalWrite(REL_RD,v_on);  
      }
      else {
        digitalWrite(REL_RD,v_off);
      }
      digitalWrite(REL_BLINK,v_on);
      delay(second_1);
      break;
    case dusk:
      Serial.println("dusk executions");
      digitalWrite(REL_BLINK,v_on);
      wait_minutes(waiting*5);
      closeDoor();
      wait_minutes(1);
      digitalWrite(resetPin,LOW);
      break;
    case between_states:
      Serial.println("interstate executions");  
      ledBlink(100,100);
      wait_minutes(1);
  }
}
