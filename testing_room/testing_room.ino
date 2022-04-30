#include <RTClib.h>
#include "math.h"
#include "Wire.h"
#include "rgb_lcd.h"

RTC_DS1307 rtc;// SDA is connecteed to A4, SCL to A5, power to A3, gnd to gnd
rgb_lcd lcd;

#define inv_rel 1
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
int v_on;
int v_off;

int ledBlink(int SEC_ON, int SEC_OFF){
  for (int i = 0; i <= 5; i++){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(SEC_ON);
    digitalWrite(LED_BUILTIN,LOW);
    delay(SEC_OFF);
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

void printTimes(int r, int g, int b, const char *moment, DateTime now, int lightval){
      lcd.setRGB(r,g,b);
      if (test_mode > 0){
        lcd.print(moment);
        lcd.print("\n");
        lcd.print("time is: ");
        lcd.print(now.year());
        lcd.print(" - ");
        lcd.print(now.day());
        lcd.print(" - ");
        lcd.print(now.month());
        lcd.print(" ");
        lcd.print(now.hour());
        lcd.print(":");
        lcd.print(now.minute());
        lcd.print("\n");
        lcd.print("sunrise and sunset is at: ");
        lcd.print(srise/60);
        lcd.print(":");
        lcd.print(srise%60);
        lcd.print(" , ");
        lcd.print(sset/60);
        lcd.print(":");
        lcd.print(sset%60);
        lcd.print("\n");
        lcd.print("measured light value is: "):
        lcd.print(lightval);
        lcd.print("\n");
        lcd.print(" door open = ");
        lcd.print(digitalRead(CL_SENS));
        lcd.print("\n");
        lcd.println(count);
      }
}

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16,2);
  digitalWrite(resetPin,HIGH);
  pinMode(resetPin,OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
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
}

void loop() {
  now = DateTime(2022. 4, 26, 10,10,00);
  ledBlink(100,100);
  printTimes(255,0,0, "day",now, 0);
  openDoor();
  delay(2000);
  ledBlink(1000,1000);
  printTimes(0,0,255, "night", now, 0);
  closeDoor();
  delay(2000);
  // put your main code here, to run repeatedly:

}
