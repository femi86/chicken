
const int OPEN = 8;
const int CLOSE = 9;
const int LIGHT_SENSOR = A2;
const int REL_LIGHT = 4;
const int CL_SENS = 5; // bridges ground to digital pin 5


int spd_up = 300; // speed of the motor to open
int spd_down = 100; // speed of the motor to close
int time_close = 30; // time it takes to close the door, in seconds
int DAWN = 30;
int DAY = 300;
int count = 0;
int day_set = 0;
int INTEG  = 30;
int RPT = 15;


int wait_minutes(int minutes){
  for (int i=0; i<= minutes*59; i++){
    delay(1000);
  }
}

int averageLight(){
  int result = 0;
  for (int i = 0; i <= INTEG; i++){
    result += analogRead(LIGHT_SENSOR);
    delay(1000); 
  }
  return result /= INTEG;
}

int door(int motor, spd){
  analogWrite(motor,spd);
  for (int i = 0; i <= time_close*10; i++){
    delay(100);
    if (digitalRead(CL_SENS)<1){
      delay(50);      
      break;
    }
  }
  analogWrite(motor,0);
  digitalWrite(REL_LIGHT,!digitalRead(REL_LIGHT));
}

int ledBlink(int SEC_ON, int SEC_OFF){
  for (int i = 0; i <= 5; i++){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(SEC_ON);
    digitalWrite(LED_BUILTIN,LOW);
    delay(SEC_OFF);
  }
}

enum timeofday{night, dawn, day, dusk};

int checktime(){
  int light_val = averageLight();
  if (light_val < DAWN && day_set == 0){
    return night;
  }
  if (light_val > DAWN && day_set == 0){
    return dawn;
  }
  if (light_val > DAY){
    return day;
  }
  if (light_val < DAWN && day_set == 1){
    return dusk;
  }
}

void(* resetFunc) (void) = 0; //declare reset function at address 0, to reset the arduino at the end of one loop (night)

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(OPEN, OUTPUT);
  pinMode(CLOSE, OUTPUT);
  pinMode(LIGHT_SENSOR,INPUT);
  pinMode(REL_LIGHT,OUTPUT);
  pinMode(CL_SENS, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch(checktime()){
    case night:
      ledBlink(300, 1500);
      delay(5000);
      count = 0;
      break;
    case dawn:
      count +=1;
      ledBlink(300,300);
      delay(5000);
      if (count==RPT){
        door(OPEN);
        delay(500);
      }
      break;
    case day:
      day_set == 1;
      ledBlink(1500,1500);
      wait_minutes(2);
      break;
    case dusk:
      wait_minutes(3);
      door(CLOSE);
      resetFunc();
      break;
  }
}
