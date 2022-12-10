#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096) OLD: 110, 100 
#define SERVOMAX  453 // This   is the 'maximum' pulse length count (out of 4096) OLD: 550
#define USMIN  550 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 - 550 w/center of 1465-1487
#define USMAX  2425 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600 - 2425 w/center of 1465-1487
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
//int sweep[19] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};
//int sweep2[19] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};
int sweep[15] = {90, 80, 70, 100, 110, 120, 130, 140, 150, 160, 170, 180};
int sweep2[16] = {90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 90, 80, 70};
uint8_t angle_odd = 90;
uint8_t angle_even = 90;
uint8_t angle_max = 180;
uint8_t angle_min = 0;
uint8_t num1 = 90;
uint8_t num2 = 90;

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

  for(uint8_t p=0; p<5; p++){
    num1 = 90; num2 = 90;
    
    uint16_t pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    
    pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
    delay(1500);
    Serial.print(num1);
    Serial.print(" ");
    Serial.println(num2);
  }

  for(uint8_t p=0; p<5; p++){
    num1 = 180; num2 = 180;
    
    uint16_t pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    
    pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
    delay(1500);
    Serial.print(num1);
    Serial.print(" ");
    Serial.println(num2);
  }
  
  
  for(uint8_t i=0; i<15; i++){

    // Bend in one direction
    for(uint8_t j=0; j<16; j++){

      num1 = sweep[i];
      num2 = sweep2[j];
      
      uint16_t pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
      pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
      
      pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
      pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
      delay(1500);
      Serial.print(num1);
      Serial.print(" ");
      Serial.println(num2);
      delay(200);
    }

    // Neutral for one second
    num1 = 90; num2 = 90;
    
    uint16_t pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    
    pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
    delay(1500);
    Serial.print(num1);
    Serial.print(" ");
    Serial.println(num2);
    delay(200);


    // Bend the other way
    for(uint8_t j=0; j<16; j++){

      num1 = sweep[i];
      num2 = sweep2[j];
      
      uint16_t pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
      pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
      
      pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
      pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
      delay(1500);
      Serial.print(num2);
      Serial.print(" ");
      Serial.println(num1);
      delay(200);
    }
  }

}
