#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#include <Wire.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096) other: 100, other: 150
#define SERVOMAX  453// This is the 'maximum' pulse length count (out of 4096) other: 453 for the 1440s: 433, other: 600
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 15;
uint16_t test = 0;
uint8_t num1 = 90;
uint8_t num2 = 90;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

uint8_t inverse(uint8_t num){
  return 180 - num;
}

void loop() {

  uint16_t pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, pulselen);

  pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(1, 0, pulselen);

  pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(2, 0, pulselen);

  pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(3, 0, pulselen);

  pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(4, 0, pulselen);

  pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(5, 0, pulselen);

  pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(6, 0, pulselen);

  pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(7, 0, pulselen);

  pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(8, 0, pulselen);

  pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(9, 0, pulselen);

  pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(10, 0, pulselen);

  pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(11, 0, pulselen);

  pulselen = map(inverse(num1), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(12, 0, pulselen);

  pulselen = map(num1, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(13, 0, pulselen);

  pulselen = map(inverse(num2), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(14, 0, pulselen);

  pulselen = map(num2, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(15, 0, pulselen);

}
