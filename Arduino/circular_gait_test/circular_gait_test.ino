#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096) USED TO BE 110
#define SERVOMAX  453 // This   is the 'maximum' pulse length count (out of 4096) USED TO BE 550
#define USMIN  550 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 - 550 w/center of 1465-1487
#define USMAX  2425 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600 - 2425 w/center of 1465-1487
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;

// This is the (bad) bilinear interpolation
//uint8_t num1[10] = {150, 70, 40, 0, 0, 50, 90, 150, 170, 170};
//uint8_t num1_r[10] = {50, 90, 150, 170, 170, 150, 70, 40, 0, 0};
//uint8_t num2[10] = {90, 180, 90, 130, 60, 90, 90, 90, 90, 60};
//uint8_t num2_r[10] = {90, 90, 90, 90, 60, 90, 180, 90, 130, 60};

// This is nearest-neighbors interpolation
//uint8_t num1[10] = {90, 70, 80, 80, 70, 90, 130, 130, 130, 130};
//uint8_t num1_r[10] = {90, 130, 130, 130, 130, 90, 70, 80, 80, 70};
//uint8_t num2[10] = {85, 75, 85, 85, 75, 85, 120, 120, 120, 120};
//uint8_t num2_r[10] = {85, 120, 120, 120, 120, 85, 75, 85, 85, 75};

// This is the better bilinear interpolation
uint8_t num1[10] = {122, 100, 20, 20, 10, 52, 80, 52, 174, 80};
uint8_t num1_r[10] = {52, 80, 52, 174, 80, 122, 100, 20, 20, 10};
uint8_t num2[10] = {165, 180, 180, 0, 60, 25, 0, 0,  54, 144};
uint8_t num2_r[10] = {25, 0, 0,  54, 144, 165, 180, 180, 0, 60};

// This is from a zero policy
uint8_t num1[10] = {90, 160, 180, 180, 160, 90, 80, 80, 80, 80};
uint8_t num1_r[10] = {90, 80, 80, 80, 80, 90, 160, 180, 180, 160};
uint8_t num2[10] = {90, 90, 80, 80, 90, 90, 150, 180, 180, 150};
uint8_t num2_r[10] = {90, 150, 180, 180, 150, 90, 90, 80, 80, 90};


uint8_t angle_odd = 90;
uint8_t angle_even = 90;
uint8_t angle_max = 180;
uint8_t angle_min = 0;


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

  for(uint8_t p=0; p<10; p++){
    

    uint16_t pulselen = map(num1[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(inverse(num1[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(num2[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(inverse(num2[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);

    pulselen = map(num1_r[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(inverse(num1_r[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(num2_r[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(inverse(num2_r[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);

    pulselen = map(num1[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(inverse(num1[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(num2[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(inverse(num2[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);

    pulselen = map(num1_r[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(inverse(num1_r[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(num2_r[p], 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(inverse(num2_r[p]), 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
    
    delay(100);
  }
  

}
