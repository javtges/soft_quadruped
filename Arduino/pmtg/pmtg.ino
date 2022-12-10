#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#include <Wire.h>

#include <NoDelay.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  453 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
noDelay gait(300);
uint8_t count = 1;


int time_ac = 100;
int time_bd = 100;
uint16_t Ary[] = {90, 90, 90, 90, 90, 90, 90, 90};

uint8_t inverse(uint8_t num){
  return 180 - num;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1);

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

  while (!Serial.available());
  String params = Serial.readString();
  Serial.println(params);
  uint8_t i=0, j=0;
  while ( j<params.length()){
    if (params.charAt(j)==',' || params.charAt(j)=='[' || params.charAt(j)==']' || params.charAt(j)==' ' || params.charAt(j)=='\''){
    }
    else {
      uint8_t b=0;
      while( b < 4){
        if (params.charAt(j+b)=='.'){
          break;
        } 
        else{
          b++;
        }
      }
      String num = params.substring(j, j+b);
      Ary[i] = num.toInt();
//      Serial.println(Ary[i]);
      i++;
      j=j+b+2;
     }
  j++;
  }

  for (int k=0; k<8; k++){
    Serial.print("Param?");
    Serial.println(Ary[k]);
  }
  uint16_t pulselen = map(Ary[0], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
  pulselen = map(inverse(Ary[0]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
  
  pulselen = map(Ary[1], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
  pulselen = map(inverse(Ary[1]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
  
  pulselen = map(Ary[2], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
  pulselen = map(inverse(Ary[2]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
  pulselen = map(Ary[3], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
  pulselen = map(inverse(Ary[3]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
  pulselen = map(Ary[4], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
  pulselen = map(inverse(Ary[4]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
  pulselen = map(Ary[5], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
  pulselen = map(inverse(Ary[5]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
  pulselen = map(Ary[6], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
  pulselen = map(inverse(Ary[6]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
  pulselen = map(Ary[7], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
  pulselen = map(inverse(Ary[7]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
  
}
  
