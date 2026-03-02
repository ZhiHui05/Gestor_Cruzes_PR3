#include <ESP32Servo.h>

Servo myservo;
const int SERVO_PIN = 4;

void setup() {
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN);
}

void loop() {
  myservo.write(90);
}