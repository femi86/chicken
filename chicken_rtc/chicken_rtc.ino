#include <RTClib.h>
#include "math.h"
#include "Wire.h"
#include "rgb_lcd.h"
#include <stdio.h>

RTC_DS1307 rtc;// SDA is connecteed to A4, SCL to A5, power to A3, gnd to gnd
rgb_lcd lcd; // SDA is connecteed to A4, SCL to A5, power to A3, gnd to gnd

// I/O definition
#define REL_LAMP 2 // def 2 this is the switch to turn on the heating lamp, on relay 1
#define REL_BLINK 3 // the relay for the blinker
#define REL_OP 4 //def 4 this is the command to open the door, on relay 3
#define REL_CL 5 // def 5 this is the command to close the door, on relay 2
#define CL_SENS 6 // def 6 the switch, that bridges ground to digital pin 6 to detect when it is closed (0 is closed, 1 is open)
#define adjust_time 7 // the pin to short when adjusting the time
#define REL_RD 8 // def 8 this is the switch to turn on the radio and the motion sensor light, on relay 4
#define OP_SENS 9 // def 9 this is the limit switch that bridges ground to digital pin 6 to detecto when it is the limit for open (1 is still opening, 0 is open)
#define additional_power 10 // the pin to provide additional power
#define resetPin 12 // reset pin for manual reset

#define TEMP_SENS A1 // def a1
#define LIGHT_SENSOR A2 // def a2

// operative parameters
#define test_mode 0 // if in test mode, value is greater than 0
// functon params
#define sens 1 // def 1, this is the sensitivity for the hysteresis evaluation, which is the "noise" on the sensor, it has to be determined manually depending on the position of the sensor. try it a few times with the sensor_det code
#define criticalT 2 // def 2, the critical temperature (in °C) below which the lamp is turned on
#define DAWN 80// this is the value at which the door starts opening (it is the same as dusk)
#define DAY 600 // def 700, this is the value at which it is definitely day
#define buff 120// this is how long the window for dawn and sunset evaluation should be, in minutes

// relay params
#define inv_rel 1 // def 1, if the relay is turned on with ground, otherwise 0
#define T_MOT 20// def 27, the time it takes the motor to close in seconds
#define second_1 1000 // def 1000, one second, can be shortened for debug 

int lampState = 0;
// location params

double lat {47.3739}; // your latitude
double lon  {8.5451}; // your longitude
int tz {2}; // your timezone vs UTC.
int spring {86}; // the day number when daylight saving comes in this year (2022)
int fall {304}; // the day number when daylight saving goes away this year (2022)
int v_off;
int v_on;

// script speed params
int INTEG {15}; // def 15, how many iterations for the sensor averaging
const int RPT {5}; // def 15, how many measurements before deciding it is indeed dawn (to exit the first loop)
int average_time {500}; // def 500, this defines how fast the sensor integrates, in ms
int waiting {2}; // time to wait within a state

// init the variables used in  script
int count {0};
int day_ct {0};

const double rad {57296}; // denominator for radians
const double deg {57296}; // numerator for degree
int srise {0};
int sset {0};

// function definition, mathematical
int check_date (DateTime now) {
  int day_t = now.day();
  int feb = 28;
  if ( (now.year() % 4 == 0 && now.year() % 100 !=0 ) || (now.year() % 400 == 0)) {
    feb = 29;
  }
  switch (now.month()){
    case 2:
      day_t += 31;
      break;
    case 3:
      day_t += 31 + feb;
      break;
    case 4:
      day_t += 31 * 2 + feb;
      break;
    case 5:
      day_t += 31 * 2 + feb + 30;
      break;
    case 6:
      day_t += 31*3 + feb + 30;
      break;
    case 7:
      day_t += 31*3 + feb + 30*2;
      break;
    case 8:
      day_t += 31*4 + feb + 30*2;
      break;
    case 9:
      day_t += 31*4 + feb + 30*3;
      break;
    case 10:
      day_t += 31*5 + feb + 30*3;
      break;
    case 11:
      day_t += 31*6 + feb + 30*3;
      break;
    case 12:
      day_t += 31*6 + feb + 30*4;
      break;
  }
  return day_t;
}

int sun_time(double LAT, double LON, int TZ, DateTime now){
    int day_y = now.unixtime() / 86400L;
    int day_t = check_date(now);
    if (day_t < spring || day_t > fall){
      TZ = TZ - 1;
    }
    double j_day = double(day_y) + 25569.0 + 2415018.5 + (now.hour()/24.0 + now.minute()/1440.0) - double(TZ)/24.0; // the 25569 is the time in days between excel's 0 and unix 
    double j_cen = (j_day - 2451545)/36525; //2451545)/36525
    double moe = 23 + (26 + ((double(21.448) - j_cen * (double(46.815) + j_cen * (double(0.00059) - 
                            (j_cen*double(0.001813))))))/double(60))/double(60);
    
    double gmas = double(357.52911) + j_cen * (double(35999.05029) - (double(0.0001537) * j_cen));
    
    double sec = sin(radians(double(gmas)))*(double(1.914602) - j_cen * (double(0.004817) + (double(0.000014) * j_cen))) + 
                  sin(radians(2*gmas))*(double(0.019993) - (double(0.000101)*j_cen)) + 
                  sin(radians(double(3*gmas)))*double(0.000289);
    
    double gml = fmod(double(280.46646) + j_cen * (double(36000.76983) + (j_cen * double(0.0003032))),double(360.00));
    double stl = gml + sec;
    double oc = moe + double(0.00256) * cos(radians((double(125.04)-double(1934.136)*j_cen))); // degrees
    double sal = stl - double(0.00569)-(double(0.00478)*sin(radians((double(125.04) -(double(1934.136) * j_cen))))); // degrees
    double sd = degrees(asin(sin(double(radians(oc)))*sin(double(radians(sal))))); // degrees
    double eeo = double(0.016708634) - (j_cen * (double(0.000042037) + (double(0.0000001267) * j_cen)));
    double y = tan(double(radians(oc)/2.0))*tan(double(radians(oc)/2.0));
    double eqtime = 4.0 * degrees(y * sin(double(2.0*radians(gml))) - 
                                        (2.0*eeo*sin(double(radians(gmas)))) + 
                                        (4.0*eeo*y*sin(double(radians(gmas))) * cos(double(2.0*radians(gml)))) -
                                        (double(0.5)*y*y*sin(double(4.0*radians(gml)))) -
                                        (double(1.25)*eeo*eeo*sin(double(2.0*radians(gmas))))); // minutes
    int noon = 720 - 4*LON - eqtime + TZ*60; // minutes
    double ha = degrees(acos(cos(radians(double(90.833)))/(cos(radians(LAT)) * 
            cos(radians(sd))) - tan(radians(LAT)) * tan(radians(sd)))); // degrees
    srise = noon - ha * 4;
    sset = noon + ha * 4;
    }


int averageTemp(int integ, int average_t){
  int tmp = 0;
  for (int i = 1; i<=integ; i++){
    tmp += analogRead(TEMP_SENS);
    delay(average_t);
  }
  int a = tmp/integ;
  float R = 1023.0/a-1.0;
  R = 100000*R;
  int B = 3975;
  float temp = 1.0/(log(R/100000)/4299+1/298.15)-273.15;
  //float R = (1023-a)*10000/a;
  //float temp = 1/(log(R/10000)/B+1/298.15)-273.15;
  int tempInt =(int)round(temp);
  return tempInt;
}

int averageLight(int integ, int average_t){
  int result = 0;
  for (int i = 1; i <= integ; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(average_t); 
  }
  delay(second_1 * 5);
  for (int i = 1; i <= integ; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(average_t); 
  }
  return result /= (integ*2);
}

// operational functions

int TempLamp(){
  if (averageTemp(INTEG, average_time) <= criticalT){
    digitalWrite(REL_LAMP,v_on);
    delay(second_1);
    lampState = 1;
  }
  else {
    digitalWrite(REL_LAMP,v_off);
    delay(second_1);
    lampState = 0;
  }
}

enum timeofday{night, dawn, day_, sundown, dusk, between_states};

int checktime(){
  DateTime now = rtc.now();
  int light_val = averageLight(INTEG, average_time);
  int tm_mins = (now.hour()*60)+now.minute();
  
  if ((tm_mins > (sset+buff)) || (tm_mins < srise - buff)){
    day_ct = 0;
    return night;
  }
  else if ((tm_mins >= srise - buff) && (tm_mins <= (srise + buff))){
    if (light_val > DAWN){
      if (count == RPT){
        count = RPT + 1;
      }
      else {
        count += 1;  
      }
      day_ct = 0;
      return dawn;
    }
    else {
      count = 0;
      day_ct = 0;
      return night;
    }
  }
  else if ((tm_mins > (srise + buff)) && (tm_mins < (sset - buff))){
    day_ct += 1;
    return day_;
  }
  else if (tm_mins >= (sset - buff)){
    if (light_val > DAWN){
      day_ct = 0;
      return sundown;
    }
    else {
      if (digitalRead(CL_SENS) == 1){
        day_ct = 0;
        return dusk;
      }
      else {
        day_ct = 0;
        return night;
      }
    }
  }
  else {
    return between_states;
  }
}

int Door(const char* state){
  int SENS;
  int REL;
  int FREE;
  int END;
  int cnt = 0;
  if (state=="close"){
    SENS = CL_SENS;
    REL = REL_CL;
    FREE = 1;
    END = 0;
    }
  else if (state=="open"){
    SENS = OP_SENS;
    REL = REL_OP;
    FREE = 0;
    END = 1;
    }
  if (digitalRead(SENS) == FREE){
      while (digitalRead(SENS != END)){
        digitalWrite(REL,v_on);
	delay(50);
      }
      digitalWrite(REL,v_off);
      delay(second_1);
    }
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
    delay(second_1*2);
  }
  digitalWrite(REL_BLINK,v_off);
  delay(second_1*4);
}

int doorState(){
  int door_status;
  // check if closed
  if (digitalRead(CL_SENS) == 0){
      door_status = 0;
  }
  // check if open
  else if (digitalRead(OP_SENS) == 1){
      door_status = 1;
  }
  return door_status;
}

void printTimes(int r, int g, int b, DateTime now, int lightval, int tempval){	
      int doorStatus = doorState();
      lcd.setRGB(r,g,b);
      delay(1000);
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      char buff1[16];
      char buff2[16];
      snprintf (buff1, sizeof(buff1), "%02d:%02d L%03dD%dT%02d",now.hour(),now.minute(),lightval,doorStatus,tempval);
      snprintf (buff2, sizeof(buff2), "SR%02d:%02dSD%02d:%02d",srise/60,srise%60,sset/60,sset%60);
      lcd.setCursor(0,0);
      lcd.print(buff1);
      lcd.setCursor(0,1);
      lcd.print(buff2);
      char buffer1[57];
      sprintf (buffer1, "it is now %02d:%02d, sunrise is at %02d:%02d and sundown at %02d:%02d",now.hour(),now.minute(),srise/60,srise%60,sset/60,sset%60);
      delay(500);
      Serial.println(buffer1);
      char buffer2[59];
      sprintf (buffer2, "light value is %03d, the door is %d, the temperature is %+02d C, lamp is %d",lightval,doorStatus,tempval,lampState);
      delay(500);
      Serial.println(buffer2);
      lcd.home();
}

void(* resetFunc) (void) = 0; //declare reset function at address 0, to reset the arduino at the end of one loop (night)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  digitalWrite(resetPin,HIGH);
  pinMode(resetPin,OUTPUT);
  pinMode(additional_power, OUTPUT);
  digitalWrite(additional_power, HIGH);
  digitalWrite(adjust_time,HIGH);
  lcd.begin(16,2);
  if (! rtc.begin()) {
    while(true){
      ledBlink(50,50);
    }
  }
  if (! rtc.isrunning()){
    ledBlink(500,100);
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(REL_OP,OUTPUT);
  pinMode(REL_CL,OUTPUT);
  pinMode(REL_RD,OUTPUT);
  pinMode(REL_LAMP,OUTPUT);
  pinMode(REL_BLINK,OUTPUT);
  pinMode(CL_SENS,INPUT_PULLUP);
  pinMode(OP_SENS,INPUT);
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
  int lightval = averageLight(5,50);
  DateTime now = rtc.now();
  int tempval = averageTemp(5,500); 
  int check = checktime();
  //Serial.println(tempval);
  //Serial.println(now);
  if (test_mode > 0){
    int INTEG = 5; // def 10, how many iterations for the sensor averaging
    const int RPT = 2; // def 15, how many measurements before deciding it is indeed dawn (to exit the first loop)
    int average_time = 100;
    int average_hyst = 2;
    int waiting = 1;
  }
  switch(check){
    case night:
      Serial.println("Arduino mode Night");
      printTimes(0,0,0, now, lightval, tempval);
      ledBlink(300,1500);
      TempLamp();
      nightBlink(40);
      digitalWrite(REL_RD, v_off);
      Door("close");
      break;
    case dawn:
      Serial.println("Arduino mode Dawn");
      printTimes(255,192,203, now, lightval, tempval);
      ledBlink(300,300);
      nightBlink(10);
      digitalWrite(REL_RD, v_off);
      switch(count){
        case RPT:
          Door("open");
          digitalWrite(REL_LAMP,v_off);
          break;
        case RPT+1:
          Door("open");
          break;
        default:
          delay(second_1*15);
          break;
      }
      break;
    case day_:
      Serial.println("Arduino mode Day");
      printTimes(255,255,0, now, lightval, tempval);
      ledBlink(1500,1500);
      if (lightval > DAWN);{
        Door("open");  
      }
      wait_minutes(waiting);
      digitalWrite(REL_RD,v_off);
      break;
    case sundown:
      Serial.println("Arduino mode Sundown");
      printTimes(160,32,240, now, lightval, tempval);
      ledBlink(600,300);
      if (digitalRead(CL_SENS) == 1){
        digitalWrite(REL_RD,v_on);  
      }
      else {
        digitalWrite(REL_RD,v_off);
      }
      digitalWrite(REL_BLINK,v_on);
      delay(second_1);
      break;
    case dusk:
      Serial.println("Arduino mode Dusk");
      printTimes(0,0,139, now, lightval, tempval);
      digitalWrite(REL_BLINK,v_on);
      //wait_minutes(1);
      delay(second_1);
      Door("close");
      delay(second_1);
      digitalWrite(REL_RD,v_off);
      //wait_minutes(1);
      digitalWrite(resetPin,LOW);
      // resetFunc();
      break;
    case between_states:
      Serial.println("Arduino mode interstates");  
      ledBlink(100,100);
      wait_minutes(1);
  }
}
