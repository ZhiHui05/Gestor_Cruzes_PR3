/*
 Test sensores infrarrojos TCRT5000
 ESP32-S3
*/
#include <Arduino.h>
/* ---------- PINES SENSORES ---------- */
#define SENSOR_IZQ 15
#define SENSOR_DER 16
/* ---------- SETUP ---------- */
void setup() {
 Serial.begin(115200);
 delay(1000);
 Serial.println("=== TEST TCRT5000 ESP32-S3 ===");
 pinMode(SENSOR_IZQ, INPUT);
 pinMode(SENSOR_DER, INPUT);
}
/* ---------- LOOP ---------- */
void loop() {
 int izq = digitalRead(SENSOR_IZQ);
 int der = digitalRead(SENSOR_DER);
 Serial.print("Sensor IZQ: ");
 Serial.print(izq);
 Serial.print(" | Sensor DER: ");
 Serial.println(der);
 delay(200);
}