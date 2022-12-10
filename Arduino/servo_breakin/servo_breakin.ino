#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  110 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This   is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
int sweep[19] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};
int sweep2[19] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};
uint8_t angle_odd = 90;
uint8_t angle_even = 90;
uint8_t angle_max = 180;
uint8_t angle_min = 0;
uint8_t num1 = 180;
uint8_t num2 = 180;
bool hasrun = false;

uint8_t inverse(uint8_t num){
  return 180 - num;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
//  Serial.println(servonum);

  if(!hasrun){

    for(uint8_t i = 0; i<10; i++){
      num1 = 90; num2 = 90;
      uint16_t pulselen = map(90, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
      pulselen = map(90, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
      
      pulselen = map(90, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
      pulselen = map(90, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
      delay(2000);  
  
      num1 = 180; num2 = 180;
      pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
      pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
      
      pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
      pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
      delay(2000);
    }
    hasrun = true;
    num1 = 90; num2 = 90;
    uint16_t pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    
    pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
  }
  
}
