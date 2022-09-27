#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#include <Wire.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  110 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;

//float p_0 = 0;
//float p_1 = 180;
//float p_2 = 180;
//float p_3 = 0;
//float p_4 = 0;
//float p_5 = 180;
//float p_6 = 180;
//float p_7 = 0;
//float p_8 = 0;
//float p_9 = 180;
//float p_10 = 180;
//float p_11 = 0;
//float p_12 = 0;
//float p_13 = 180;
//float p_14 = 180;
//float p_15 = 0;

float p_0 = 90;
float p_1 = 90;
float p_2 = 90;
float p_3 = 90;
float p_4 = 90;
float p_5 = 90;
float p_6 = 90;
float p_7 = 90;
float p_8 = 90;
float p_9 = 90;
float p_10 = 90;
float p_11 = 90;
float p_12 = 90;
float p_13 = 90;
float p_14 = 90;
float p_15 = 90;

//float s_0 = 180;
//float s_1 = 0;
//float s_2 = 0;
//float s_3 = 180;
//float s_4 = 180;
//float s_5 = 0;
//float s_6 = 0;
//float s_7 = 180;
//float s_8 = 180;
//float s_9 = 0;
//float s_10 = 0;
//float s_11 = 180;
//float s_12 = 180;
//float s_13 = 0;
//float s_14 = 0;
//float s_15 = 180;

float s_0 = 90;
float s_1 = 90;
float s_2 = 90;
float s_3 = 90;
float s_4 = 90;
float s_5 = 90;
float s_6 = 90;
float s_7 = 90;
float s_8 = 90;
float s_9 = 90;
float s_10 = 90;
float s_11 = 90;
float s_12 = 90;
float s_13 = 90;
float s_14 = 90;
float s_15 = 90;

int time_ac = 100;
int time_bd = 100;
uint16_t Ary[] = {9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000};

void AC(int a_0, int a_1, int a_2, int a_3, int c_8, int c_9, int c_10, int c_11){
    a_0 = constrain(a_0, 0, 180);
    a_1 = constrain(a_1, 0, 180);
    a_2 = constrain(a_2, 0, 180);
    a_3 = constrain(a_3, 0, 180);

    c_8 = constrain(c_8, 0, 180);
    c_9 = constrain(c_9, 0, 180);
    c_10 = constrain(c_10, 0, 180);
    c_11 = constrain(c_11, 0, 180);
    
    uint16_t pulselen = map(c_8, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(c_9, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(c_10, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(c_11, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);

    pulselen = map(a_0, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(a_1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(a_2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(a_3, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
}

void BD(int b_4, int b_5, int b_6, int b_7, int d_12, int d_13, int d_14, int d_15){
    b_4 = constrain(b_4, 0, 180);
    b_5 = constrain(b_5, 0, 180);
    b_6 = constrain(b_6, 0, 180);
    b_7 = constrain(b_7, 0, 180);

    d_12 = constrain(d_12, 0, 180);
    d_13 = constrain(d_13, 0, 180);
    d_14 = constrain(d_14, 0, 180);
    d_15 = constrain(d_15, 0, 180);
    
    uint16_t pulselen = map(b_4, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(b_5, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(b_6, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(b_7, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);

    pulselen = map(d_12, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(d_13, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(d_14, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(d_15, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t angle_odd = 90;
  uint8_t angle_even = 90;
  uint8_t angle_max = 180;
  uint8_t angle_min = 0;

  if (Serial.available() > 0){
    
    AC(90, 90, 90, 90, 90, 90, 90, 90);
    BD(90, 90, 90, 90, 90, 90, 90, 90);
    
    String params = Serial.readString();
    Serial.println("Read a new set of params");
    uint8_t i=0, j=0;
    while ( j<params.length()){
      if (params.charAt(j)==',' || params.charAt(j)=='[' || params.charAt(j)==']' || params.charAt(j)==' ' || params.charAt(j)=='\''){
      }
      else {
        String num = params.substring(j, j+5);
        Ary[i] = num.toInt();
        i++;
        j= j+5;
        }
    j++;
    }
    delay(100);
  }

  Serial.println(Ary[0]);
  p_0 = Ary[0]/100;
  Serial.println(p_0);
  p_1 = Ary[1]/100;
  p_2 = Ary[2]/100;
  p_3 = Ary[3]/100;
  p_4 = Ary[4]/100;
  p_5 = Ary[5]/100;
  p_6 = Ary[6]/100;
  p_7 = Ary[7]/100;
  p_8 = Ary[8]/100;
  p_9 = Ary[9]/100;
  p_10 = Ary[10]/100;
  p_11 = Ary[11]/100;
  p_12 = Ary[12]/100;
  p_13 = Ary[13]/100;
  p_14 = Ary[14]/100;
  p_15 = Ary[15]/100;


  s_0 = 2*(90 - p_0) + p_0;
  s_1 = 2*(90 - p_1) + p_1;
  s_2 = 2*(90 - p_2) + p_2;
  s_3 = 2*(90 - p_3) + p_3;
  s_4 = 2*(90 - p_4) + p_4;
  s_5 = 2*(90 - p_5) + p_5;
  s_6 = 2*(90 - p_6) + p_6;
  s_7 = 2*(90 - p_7) + p_7;
  s_8 = 2*(90 - p_8) + p_8;
  s_9 = 2*(90 - p_9) + p_9;
  s_10 = 2*(90 - p_10) + p_10;
  s_11 = 2*(90 - p_11) + p_11;
  s_12 = 2*(90 - p_12) + p_12;
  s_13 = 2*(90 - p_13) + p_13;
  s_14 = 2*(90 - p_14) + p_14;
  s_15 = 2*(90 - p_15) + p_15;

  Serial.println("p0 s0");
  Serial.println(p_0);
  Serial.println(s_0);

  AC(p_0, p_1, p_2, p_3, p_8, p_9, p_10, p_11);
  delay(200);
  BD(p_4, p_5, p_6, p_7, p_8, p_9, p_10, p_11);
  delay(200);
  AC(s_0, s_1, s_2, s_3, s_8, s_9, s_10, s_11);
  delay(200);
  BD(s_4, s_5, s_6, s_7, s_8, s_9, s_10, s_11);
  delay(200);

}
