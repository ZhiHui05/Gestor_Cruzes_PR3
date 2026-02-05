/*
 Test de motores
 ESP32-S3 + Mini L298N
*/
#include <Arduino.h>
/* ---------- PINES PUENTE H ---------- */
// Motor izquierdo: IN1 (GPIO 4), IN2 (GPIO 5)
#define IN1 4
#define IN2 5
// Motor derecho: IN3 (GPIO 6), IN4 (GPIO 7)
#define IN3 6
#define IN4 7
/* ---------- PWM ---------- */
const int PWM_FREQ = 5000; // 5 kHz
const int PWM_RES = 8; // 0–255
const int VELOCIDAD = 180; // (≈70 % de potencia)
/* ---------- SETUP ---------- */
void setup() {
 Serial.begin(115200);
 delay(1000);
 Serial.println("=== TEST MOTORES ESP32-S3 ===");
 //Cada pin se configura como salida PWM usando ledcAttach()
 ledcAttach(IN1, PWM_FREQ, PWM_RES);
 ledcAttach(IN2, PWM_FREQ, PWM_RES);
 ledcAttach(IN3, PWM_FREQ, PWM_RES);
 ledcAttach(IN4, PWM_FREQ, PWM_RES);
}
/* ---------- LOOP ---------- */
void loop() {
 Serial.println("Avanzar recto");
 moverMotores(VELOCIDAD, VELOCIDAD);
 // Ambos motores giran hacia delante a la misma velocidad
 delay(3000);
 Serial.println("Parar");
 moverMotores(0, 0); // Parar ambos motores
 delay(1000);
 Serial.println("Giro izquierda");
 moverMotores(0, VELOCIDAD); // Parar solo motor izquierdo
 delay(3000);
 Serial.println("Parar");
 moverMotores(0, 0);
 delay(1000);
 Serial.println("Giro derecha");
 moverMotores(VELOCIDAD, 0); // Parar solo motor derecho
 delay(3000);
 Serial.println("Parar largo");
 moverMotores(0, 0);
 delay(3000);
}

void moverMotores(int velIzq, int velDer) {
 // El avance se logra aplicando PWM en IN1 e IN3.
 // IN2 e IN4 se mantienen a 0.
 // Motor izquierdo
 ledcWrite(IN1, velIzq);
 ledcWrite(IN2, 0);
 // Motor derecho
 ledcWrite(IN3, velDer);
 ledcWrite(IN4, 0);
 Serial.printf("Vel Izq=%d | Vel Der=%d\n", velIzq, velDer);
}