/* Serial passthrough drive firmware for the tortoise bot. 
 * 
 * This set of firmware acts as a passthrough for a main drive computer.
 * Messages are sent over serial to set motor angles or request sensor
 * data. 
 * 
 * Maintainer: Jake Ketchum (jketchum@u.northwestern.edu)
 */
 
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();

// Servo motion parameters. 
#define SERVOMIN  110 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This is the 'maximum' pulse length count (out of 4096)
// TODO: This line might be wrong.
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define LED_PIN 0

double steering_angle = 0;

void setup() {

  // Boilerplate code to boot up the serial driver. 
  servo_driver.begin();
  servo_driver.setOscillatorFrequency(27000000);
  servo_driver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(400);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  process_serial();
}

void update_servos(int angles[]){
  for(int i = 0; i < 16; i++)
  {
    uint16_t pulselen = map(angles[i], 0, 255, SERVOMIN, SERVOMAX); servo_driver.setPWM(i, 0, pulselen);
  }
}

void update_led(byte state)
/*
 * Write the LED pin. 
 * Args:
 *    state (byte) - contains 0 to dissable, or 1 to enable the LED.
 */
{
  digitalWrite(LED_PIN, state);
}
