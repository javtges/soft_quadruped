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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  Serial.println(servonum);

  
  
  uint8_t angle_odd = 90;
  uint8_t angle_even = 90;
  uint8_t angle_max = 180;
  uint8_t angle_min = 0;
  if ((servonum%2)==0) { uint16_t pulselen = map(angle_even, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(servonum, 0, pulselen); }
  if (((servonum%2)==1)){ uint16_t pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(servonum, 0, pulselen);}

//  pwm.setPWM(servonum, 0, pulselen);
//  delay(200);



  servonum++;
  Serial.println("servo up");
  if (servonum > 15)  { servonum = 0;}

    delay(100);
    // Expand all legs - 0
    uint16_t pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
     delay(100);


//  Contract leg A and C - 1
    pulselen = map(-10, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(190, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(-10, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(190, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);


//    while(1);
     delay(100);
//
//  Bend A and C forward - 2
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
    delay(100);

    // Bend C forward - 3
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
     delay(100);

    //contract B - 4
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
     delay(100);

    //Contract D - 5
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
     delay(100);
    //Bend D forward - 6
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
     delay(100);

    //expand A, C, D - neutral B - 7
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
     delay(100);
    
    //contract leg B - 8
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
    delay(100);


    //Bend B forward - 9
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
     delay(100);

    //neutral A,C,D - 10
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOM0IN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
    pulselen = map(angle_odd, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
     delay(100);

//    while(1){;}

////    Contract all legs
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(0, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(1, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(2, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(3, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(4, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(5, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(6, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(7, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(8, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(9, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(10, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(11, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(12, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(13, 0, pulselen);
//    pulselen = map(angle_min, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(14, 0, pulselen);
//    pulselen = map(angle_max, 0, 180, SERVOMIN, SERVOMAX); pwm.setPWM(15, 0, pulselen);
//    delay(500);


}
