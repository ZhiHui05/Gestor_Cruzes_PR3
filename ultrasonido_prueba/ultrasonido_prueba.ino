/*
Test sensor ultrasónico HC-SR04
 ESP32-S3
*/
#include <Arduino.h>
/* ---------- PINES HC-SR04 ---------- */
#define TRIG_PIN 17
#define ECHO_PIN 18
/* ---------- SETUP ---------- */
void setup() {
 Serial.begin(115200);
 delay(1000);
 Serial.println("=== TEST HC-SR04 ESP32-S3 ===");
 pinMode(TRIG_PIN, OUTPUT);
 pinMode(ECHO_PIN, INPUT);
 digitalWrite(TRIG_PIN, LOW);
}
/* ---------- LOOP ---------- */
void loop() {
 // Pulso de disparo (10 µs)
 digitalWrite(TRIG_PIN, LOW);
 delayMicroseconds(2);
 digitalWrite(TRIG_PIN, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIG_PIN, LOW);
 // Leer duración del eco (µs)
 long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
 if (duration == 0) {
 Serial.println("Fuera de rango / sin eco");
 } else {
 // Velocidad sonido ≈ 0.034 cm/µs
 float distanciaCm = duration * 0.034 / 2.0;
 Serial.print("Distancia: ");
 Serial.print(distanciaCm);
 Serial.println(" cm");
 }
 delay(500);
}