#include <math.h>

const int REL_OP = 4; //def 4 this is the command to open the door, on relay 3
const int REL_CL = 5; // def 5 this is the command to close the door, on relay 2
const int REL_RD = 8; // def 8 this is the switch to turn on the radio and the motion sensor light, on relay 4
const int REL_LAMP = 2; // def 2 this is the switch to turn on the heating lamp, on relay 1
const int CL_SENS = 6; // def 6 the wire on the door, that bridges ground to digital pin 6 to detect when it is closed
const int LIGHT_SENSOR = A2; // def a2
const int TEMP_SENS = A1; // def a1

// functon params
const int sens = 2; // def 1, this is the sensitivity for the hysteresis evaluation, which is the "noise" on the sensor, it has to be determined manually depending on the position of the sensor. try it a few times with the sensor_det code
const int INTEG = 15; // def 10, how many iterations for the sensor averaging
const int RPT = 10; // def 15, how many measurements before deciding it is indeed dawn (to exit the first loop)
const int average_time = 500; // def 1000, this defines how fast the sensor integrates, in ms
const int criticalT = 2; // def 2, the critical temperature (in °C) below which the lamp is turned on
const int DAWN = 40; // this is the value at which the door starts opening (it is similar to dusk)
const int DAY = 550; // def 700, this is the value at which it is definitely day

// relay params
const int inv_rel = 1; // def 1, if the relay is turned on with ground, otherwise 0
const int T_MOT = 40; // def 27, the time it takes the motor to close in seconds
const int second_1 = 1000; // def 1000, one second, can be shortened for debug 
int count = 0;
int dusk_set = 0;
int day_set = 0;
int v_on;
int v_off;

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

int TempLamp(){
  if (averageTemp() <= criticalT){
    digitalWrite(REL_LAMP,v_on);
    wait_minutes(2);
  }
  else{
    digitalWrite(REL_LAMP,v_off);
    delay(second_1);
  }
}

int averageLight(){
  int result = 0;
  for (int i = 1; i <= INTEG; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(average_time); 
  }
  return result /= INTEG;
}

int check_hyst(){
  int change = 0;
  int vals[] = {0,0};
  for (int n = 0; n <= 1; n++){
    vals[n] = averageLight();
    for (int x=0; x <= 30; x++){
      delay(second_1);
    }
  }
  if (abs(vals[1] - vals[0]) > sens){
    if (vals[1]< vals[0]){
      change = -1; // this indicates light is decreasing
    }
    else if (vals[1] > vals[0]){
      change = 1; // this indicates light is increasing
    }
  }
  return change;
}

enum timeofday{night, dawn, day, sundown, dusk};

int checktime(){
  int light_val = averageLight();
  int hyst = check_hyst();
  if (hyst == 0 && light_val < DAWN && dusk_set == 0){
    return night;
  }
  else if (hyst > 0 && day_set == 0){
    return dawn;
  }
  else if (hyst == 0 && light_val > DAY){
    return day;
  }
  else if (hyst < 0 && light_val > DAWN && light_val < DAY && day_set == 1){
    return sundown;
  }
  else if (hyst <= 0 && light_val < DAWN && dusk_set == 1){
    return dusk;
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

int wait_minutes(int minutes){
  for (int n = 0; n < minutes; n++){
    for (int i = 0; i<60; i++){
      delay(second_1);
    }
  }
}

int closeDoor(){
  digitalWrite(REL_CL,v_on);
  int secs = 0;
  while (true){
      delay(100);
      secs +=1;
      if (digitalRead(CL_SENS) < 1 || secs > T_MOT*10 ){// check if door closed, or if too long
        delay(200);
        break;
    }
  }
  digitalWrite(REL_CL,v_off); // this is the automatic run
  delay(second_1);
}

int ledBlink(int SEC_ON, int SEC_OFF){
  for (int i = 0; i <= 5; i++){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(SEC_ON);
    digitalWrite(LED_BUILTIN,LOW);
    delay(SEC_OFF);
  }
}

void(* resetFunc) (void) = 0; //declare reset function at address 0, to reset the arduino at the end of one loop (night)

void setup() {
  // put your setup code here, to run once:
  
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(REL_OP,OUTPUT);
  pinMode(REL_CL,OUTPUT);
  pinMode(REL_RD,OUTPUT);
  pinMode(REL_LAMP,OUTPUT);
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
}
void loop() {
  int analogValue = averageLight();
  int check = checktime();
   
  switch(check){
    case night:
      ledBlink(300,1500);
      TempLamp();
      delay(second_1*5);
      count = 0;
      break;
    case dawn:
      count += 1;
      ledBlink(300,300);
      delay(second_1*5);
      if (count == RPT){
        openDoor(); // if it is closed, open for 40 seconds
        digitalWrite(REL_LAMP,v_off);
      }
      if (count > RPT){
        //digitalWrite(REL_RD,v_on);
        delay(second_1*5);
      }
      break;
    case day:
      day_set = 1;
      ledBlink(1500,1500);
      digitalWrite(REL_RD,v_off);
      wait_minutes(2);
      break;
    case sundown:
      ledBlink(600,300);
      digitalWrite(REL_RD,v_on);
      dusk_set = 1;
      delay(second_1);
      break;
    case dusk:
      wait_minutes(5);
      closeDoor();
      digitalWrite(REL_RD,v_off);
      delay(second_1*10);
      resetFunc();
      break;
  }
}
