#include <Arduino.h>
#include <WiFi.h>
#include <PicoMQTT.h>

/* ---------- WIFI ---------- */
const char* WIFI_SSID = "CRUCE_MQTT";
const char* WIFI_PASS = "12345678";

/* ---------- MQTT ---------- */
const char* BROKER_IP = "192.168.4.1";
const int   BROKER_PORT = 1883;
const char* VEH_ID = "veh01";

PicoMQTT::Client mqtt(BROKER_IP, BROKER_PORT, VEH_ID);

/* ---------- PINES PUENTE H ---------- */
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

#define SENSOR_IZQ 15
#define SENSOR_DER 16

#define CONTROL_LED_ROJO 8
#define CONTROL_LED_VERDE 10

#define TRIG_PIN 17
#define ECHO_PIN 18

/* ---------- PWM ---------- */
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;
const int VELOCIDAD = 180;

/* ---------- ESTADOS ---------- */
enum Estado {
  ESPERANDO_AUTORIZACION,
  AUTORIZADO
};

Estado estado = ESPERANDO_AUTORIZACION;
bool autorizado = false;

/* ---------- FUNCIONES ---------- */
void moverMotores(int velIzq, int velDer);
void stopMotores();
void enviarSolicitud();

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  delay(1000);

  /* ---- Pines ---- */
  pinMode(CONTROL_LED_ROJO, OUTPUT);
  pinMode(CONTROL_LED_VERDE, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SENSOR_IZQ, INPUT);
  pinMode(SENSOR_DER, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  /* ---- PWM ---- */
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);

  /* ---- WIFI ---- */
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.println(WiFi.localIP());

 /* ---- MQTT ---- */
  mqtt.subscribe(
    "gestor/veh01/autorizacion",
    [](const char* topic, const char* payload) {
      Serial.printf("[MQTT] %s => %s\n", topic, payload);
      autorizado = true;
    }
  );

  Serial.print("Conectando a MQTT");
  mqtt.subscribe(
    "gestor/veh01/autorizacion",
    [](const char* topic, const char* payload) {
      Serial.printf("[MQTT] %s => %s\n", topic, payload);
      autorizado = true;
    }
  );

  enviarSolicitud();


  digitalWrite(CONTROL_LED_ROJO, HIGH);
  digitalWrite(CONTROL_LED_VERDE, LOW);


}

/* ---------- LOOP ---------- */
void loop() {
  mqtt.loop();
  switch (estado) {

    case ESPERANDO_AUTORIZACION:
      stopMotores();
      if (autorizado) {
        Serial.println("Autorización recibida");
        digitalWrite(CONTROL_LED_ROJO, LOW);
        digitalWrite(CONTROL_LED_VERDE, HIGH);
        estado = AUTORIZADO;
      }
      break;

    case AUTORIZADO:
      /* ---- Ultrasonido ---- */
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      long duration = pulseIn(ECHO_PIN, HIGH, 30000);
      if (duration > 0) {
        float distancia = duration * 0.034 / 2.0;
        if (distancia < 10) {
          Serial.println("Obstáculo detectado");
          stopMotores();
          break;
        }c:\Users\VLOZCAR\Documents\Arduino\sketch_feb6d\sketch_feb6d.ino
      }

      /* ---- Sensores de línea ---- */
      int izq = digitalRead(SENSOR_IZQ);
      int der = digitalReawc:\Users\VLOZCAR\Documents\Arduino\sketch_feb6a\sketch_feb6a.ino:\PR3\arduino\sketch_feb6a\sketch_feb6a.inod(SENSOR_DER);

      if (izq && der) {
        moverMotores(VELOCIDAD, VELOCIDAD);
      }
      else if (!izq && der) {
        moverMotores(VELOCIDAD, 0);
      }
      else if (izq && !der) {
        moverMotores(0, VELOCIDAD);
      }
      else {
        stopMotores();
      }
      break;
  }

  delay(2);
}

/* ---------- FUNCIONES ---------- */

void enviarSolicitud() {
  String topic = "vehiculo/veh01/solicitud";
  mqtt.publish(topic.c_str(), "{ \"solicitud\": true }");
  Serial.println("Solicitud enviada");
}


void stopMotores() {
  moverMotores(0, 0);
}

void moverMotores(int velIzq, int velDer) {
  ledcWrite(IN1, velIzq);
  ledcWrite(IN2, 0);
  ledcWrite(IN3, velDer);
  ledcWrite(IN4, 0);
}
