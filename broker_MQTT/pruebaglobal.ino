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
  AUTORIZADO,
  SALIDA,
  TIMEOUT
};

Estado estado = ESPERANDO_AUTORIZACION;
Estado estadoPrev = ESPERANDO_AUTORIZACION;

/* ===== Eventos (bitmask) ===== */
enum Evento : uint32_t {
  EV_NONE = 0,
  EV_AUTORIZACION = 1u << 0,    // autorización recibida por MQTT
  EV_OBSTACLE = 1u << 1,        // obstáculo detectado por ultrasonido
  EV_LINE_LEFT = 1u << 2,       // sensor línea izquierda
  EV_LINE_RIGHT = 1u << 3,      // sensor línea derecha
  EV_TIMEOUT = 1u << 4
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
  Serial.print("Conectando a MQTT");
  mqtt.subscribe(
    "gestor/veh01/autorizacion",
    [](const char* topic, const char* payload) {
      Serial.printf("[MQTT] %s => %s\n", topic, payload);
      // Esperamos JSON como: { "autorizacion": true }
      if (payload && strstr(payload, "true") != nullptr) {
        autorizado_flag = true;                // callback sets flag
        eventos_callbacks |= EV_AUTORIZACION; // mark event for loop
      }
    }
  );

  enviarSolicitud();


  digitalWrite(CONTROL_LED_ROJO, HIGH);
  digitalWrite(CONTROL_LED_VERDE, LOW);


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
    Serial.printf("[FSM] %d -> %d (ev=0x%08X)\n", (int)estadoPrev, (int)estado, ev);
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
    // clear the callback event (consumed)
    eventos_callbacks &= ~EV_AUTORIZACION;
  }

  // Leer ultrasonido (solo para detección)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration > 0) {
    float distancia = duration * 0.034 / 2.0;
    last_distancia = distancia;
    if (distancia < 10.0) {
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

  return ev;
}

/* ===== 2) Determinar siguiente estado (función pura) ===== */
Estado determinarSiguienteEstado(Estado s, uint32_t ev) {
  switch (s) {
    case ESPERANDO_AUTORIZACION:
      if (ev & EV_AUTORIZACION) return AUTORIZADO;
      return ESPERANDO_AUTORIZACION;
    case AUTORIZADO:
      // Si hay timeout se podría saltar a TIMEOUT (no implementado explícitamente ahora)
      if (ev & EV_TIMEOUT) return TIMEOUT;
      // Obstacle no cambia de estado aquí, lo maneamos en acciones
      return AUTORIZADO;
    case SALIDA:
      return SALIDA;
    case TIMEOUT:
      return TIMEOUT;
  }
  return s;
}

/* ===== 3) Ejecutar estado (efectos sobre HW) ===== */
void ejecutarEstado(Estado s) {
  switch (s) {
    case EV_NONE:
    if (last_izq && last_der) {
        moverMotores(VELOCIDAD, VELOCIDAD);
      }
      else if (!last_izq && last_der) {
        moverMotores(VELOCIDAD, 0);
      }
      else if (last_izq && !last_der) {
        moverMotores(0, VELOCIDAD);
      }
      else {
        stopMotores();
      }
    break;

    case ESPERANDO_AUTORIZACION:
      stopMotores();
      digitalWrite(CONTROL_LED_ROJO, HIGH);
      digitalWrite(CONTROL_LED_VERDE, LOW);
      break;
    case AUTORIZADO:
      digitalWrite(CONTROL_LED_ROJO, LOW);
      digitalWrite(CONTROL_LED_VERDE, HIGH);
      // Si hay obstáculo, detén motores
      if (last_distancia < 10.0) {
        Serial.println("Obstáculo detectado (ejecutarEstado)");
        stopMotores();
        break;
      }

      // Seguidor de línea: misma lógica que antes pero usando lecturas almacenadas
      if (last_izq && last_der) {
        moverMotores(VELOCIDAD, VELOCIDAD);
      }
      else if (!last_izq && last_der) {
        moverMotores(VELOCIDAD, 0);
      }
      else if (last_izq && !last_der) {
        moverMotores(0, VELOCIDAD);
      }
      else {
        stopMotores();
      }
      break;
    case SALIDA:
      // aquí podríamos publicar la salida y continuar misión
      break;
    case TIMEOUT:
      // manejar timeouts/reintentos si procede
      break;
  }
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