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
  SEGUIR_LINEA,           // Sigue línea negra
  PARADO_EN_CRUCE,        // Detectó obstáculo, espera autorización
  CRUZANDO,               // Recibió autorización, cruza
  SALIDA                  // Publicó salida, vuelve a SEGUIR_LINEA
};

Estado estado = SEGUIR_LINEA;
Estado estadoPrev = SEGUIR_LINEA;

/* ===== Control de timeouts ===== */
unsigned long tEstado = 0;              // tiempo de entrada al estado actual
const unsigned long TIMEOUT_CRUZAR_MS = 5000;  // timeout para cruzar
const unsigned long TIMEOUT_SOLICITUD_MS = 2000; // reintentar solicitud cada 2s

/* ===== Eventos (bitmask) ===== */
enum Evento : uint32_t {
  EV_NONE = 0,
  EV_OBSTACLE = 1u << 0,        // obstáculo detectado por ultrasonido
  EV_LINE_LEFT = 1u << 1,       // sensor línea izquierda
  EV_LINE_RIGHT = 1u << 2,      // sensor línea derecha
  EV_AUTORIZACION = 1u << 3,    // autorización recibida por MQTT
  EV_TIMEOUT = 1u << 4          // timeout esperando autorización/cruzando
};

volatile uint32_t eventos_callbacks = EV_NONE; // puesto por callbacks (MQTT, interrupciones)

/* Bandera auxiliar: marcada por callback MQTT al recibir autorización */
volatile bool autorizado_flag = false;

/* ---------- FUNCIONES ---------- */
void moverMotores(int velIzq, int velDer);
void stopMotores();
void enviarSolicitud();
uint32_t detectarEventos();
Estado determinarSiguienteEstado(Estado s, uint32_t ev);
void ejecutarEstado(Estado s);

/* Últimas lecturas para ejecutar acciones */
int last_izq = 0;
int last_der = 0;
float last_distancia = 9999.0;

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
  Serial.println("Conectando a MQTT");
  mqtt.subscribe(
    "gestor/veh01/autorizacion",
    [](const char* topic, const char* payload) {
      Serial.printf("[MQTT] Autorización recibida: %s\n", payload);
      if (payload && strstr(payload, "true") != nullptr) {
        autorizado_flag = true;
        eventos_callbacks |= EV_AUTORIZACION;
        Serial.println("[eventos] EV_AUTORIZACION marcado");
      }
    }
  );

  // Iniciar en SEGUIR_LINEA, sin solicitar autorización aún
  digitalWrite(CONTROL_LED_ROJO, HIGH);
  digitalWrite(CONTROL_LED_VERDE, LOW);
  Serial.println("[SETUP] Iniciando en estado SEGUIR_LINEA");


}

/* ---------- LOOP ---------- */
void loop() {
  mqtt.loop();

  // 1) detectar eventos y actualizar lecturas
  uint32_t ev = detectarEventos();

  // 2) determinar siguiente estado (pura)
  Estado siguiente = determinarSiguienteEstado(estado, ev);
  if (siguiente != estado) {
    estadoPrev = estado;
    estado = siguiente;
    tEstado = millis();  // reinicia timer para el nuevo estado
    Serial.printf("[FSM] Estado %d -> %d (eventos=0x%02X, tiempo=%lu)\n",
                  (int)estadoPrev, (int)estado, ev, millis());
  }

  // 3) ejecutar acciones del estado
  ejecutarEstado(estado);

  delay(2);
}

/* ===== 1) Detectar eventos (sin acciones) ===== */
uint32_t detectarEventos() {
  uint32_t ev = EV_NONE;

  // Eventos puestos por callbacks MQTT
  if (eventos_callbacks & EV_AUTORIZACION) {
    ev |= EV_AUTORIZACION;
    eventos_callbacks &= ~EV_AUTORIZACION;
  }

  // Leer ultrasonido: detecta obstáculos (cruce)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration > 0) {
    float distancia = duration * 0.034 / 2.0;
    last_distancia = distancia;
    // Obstáculo = cruce detectado
    if (distancia < 15.0) {  // aumentado a 15cm para detectar mejor el cruce
      ev |= EV_OBSTACLE;
    }
  }

  // Sensores de línea
  int izq = digitalRead(SENSOR_IZQ);
  int der = digitalRead(SENSOR_DER);
  last_izq = izq;
  last_der = der;
  if (izq) ev |= EV_LINE_LEFT;
  if (der) ev |= EV_LINE_RIGHT;

  // Timeout: si lleva más de TIMEOUT_CRUZAR_MS en CRUZANDO sin volver a línea, error
  if (estado == CRUZANDO && (millis() - tEstado) >= TIMEOUT_CRUZAR_MS) {
    ev |= EV_TIMEOUT;
    Serial.println("[detectarEventos] TIMEOUT cruzando (5s sin completar cruce)");
  }

  return ev;
}

/* ===== 2) Determinar siguiente estado (función pura) ===== */
Estado determinarSiguienteEstado(Estado s, uint32_t ev) {
  switch (s) {
    case SEGUIR_LINEA:
      // Si detecta obstáculo (cruce), se detiene y espera autorización
      if (ev & EV_OBSTACLE) {
        return PARADO_EN_CRUCE;
      }
      return SEGUIR_LINEA;

    case PARADO_EN_CRUCE:
      // Si recibe autorización, comienza a cruzar
      if (ev & EV_AUTORIZACION) {
        return CRUZANDO;
      }
      return PARADO_EN_CRUCE;

    case CRUZANDO:
      // Si timeout durante el cruce, regresa a PARADO_EN_CRUCE para reintentar
      if (ev & EV_TIMEOUT) {
        return PARADO_EN_CRUCE;
      }
      // Si vuelve a detectar línea (EV_LINE_LEFT o EV_LINE_RIGHT después del cruce),
      // el robot ha llegado al otro lado: publica salida
      if ((ev & EV_LINE_LEFT) || (ev & EV_LINE_RIGHT)) {
        return SALIDA;
      }
      return CRUZANDO;

    case SALIDA:
      // Después de publicar salida, vuelve a SEGUIR_LINEA
      return SEGUIR_LINEA;
  }
  return s;
}

/* ===== 3) Ejecutar estado (efectos sobre HW) ===== */
void ejecutarEstado(Estado s) {
  switch (s) {
    case SEGUIR_LINEA:
      // LED VERDE: siguiendo línea normalmente
      digitalWrite(CONTROL_LED_VERDE, HIGH);
      digitalWrite(CONTROL_LED_ROJO, LOW);

      // Seguidor de línea: lógica original
      if (last_izq && last_der) {
        moverMotores(VELOCIDAD, VELOCIDAD);  // adelante recto
      }
      else if (!last_izq && last_der) {
        moverMotores(VELOCIDAD, 0);  // gira izquierda
      }
      else if (last_izq && !last_der) {
        moverMotores(0, VELOCIDAD);  // gira derecha
      }
      else {
        stopMotores();  // no ve línea
      }
      break;

    case PARADO_EN_CRUCE:
      // LED ROJO: parado en el cruce, esperando autorización
      digitalWrite(CONTROL_LED_ROJO, HIGH);
      digitalWrite(CONTROL_LED_VERDE, LOW);
      stopMotores();
      
      // Envía solicitud UNA SOLA VEZ al entrar en este estado (transición desde SEGUIR_LINEA)
      if (estadoPrev == SEGUIR_LINEA) {
        enviarSolicitud();
        Serial.println("[PARADO_EN_CRUCE] Solicitud enviada al detectar cruce");
      }
      // Espera autorización del gestor (callback MQTT)
      break;

    case CRUZANDO:
      // LED VERDE: cruzando
      digitalWrite(CONTROL_LED_VERDE, HIGH);
      digitalWrite(CONTROL_LED_ROJO, LOW);
      
      // Avanza recto (ambos motores a velocidad máxima)
      moverMotores(VELOCIDAD, VELOCIDAD);
      break;

    case SALIDA:
      // LED ROJO: cruce completado, publicar salida
      digitalWrite(CONTROL_LED_ROJO, HIGH);
      digitalWrite(CONTROL_LED_VERDE, LOW);
      stopMotores();
      
      // Publica salida (solo una vez al entrar en este estado)
      if (estadoPrev == CRUZANDO) {
        String topic = "vehiculo/veh01/salida";
        String payload = "{\"salida\": true, \"timestamp\": " + String(millis()) + "}";
        mqtt.publish(topic.c_str(), payload.c_str());
        Serial.printf("[SALIDA] Publicado en topic %s\n", topic.c_str());
      }
      // Transición inmediata a SEGUIR_LINEA en el siguiente loop
      break;
  }
}

/* ---------- FUNCIONES ---------- */

void enviarSolicitud() {
  String topic = "vehiculo/veh01/solicitud";
  String payload = "{\"solicitud\": true, \"timestamp\": " + String(millis()) + ", \"estado\": \"PARADO_EN_CRUCE\"}";
  mqtt.publish(topic.c_str(), payload.c_str());
  Serial.printf("[SOLICITUD] Topic: %s, Payload: %s\n", topic.c_str(), payload.c_str());
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