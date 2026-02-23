/*
ARREGLOS REALIZADOS:
- Timeout en RETROCESO para evitar quedarse atrapado
- Timeout en PARADO_EN_CRUCE para reintentar solicitud
- Eliminado delay(1500) bloqueante de enviarSolicitud()
- Mejorada lógica de transiciones
- LEDs correctos: rojo=retroceso+esperar, verde=seguir+cruzar
- Comentarios breves añadidos
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PicoMQTT.h>

/* ---------- WIFI ---------- */
const char* WIFI_SSID = "CRUCE_MQTT";
const char* WIFI_PASS = "12345678";

/* ---------- MQTT ---------- */
const char* BROKER_IP = "192.168.4.1";
const int BROKER_PORT = 1883;
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
const int PWM_RES = 8;
const int VELOCIDAD = 210;

/* ---------- ESTADOS ---------- */
enum Estado {
SEGUIR_LINEA, // Sigue línea negra
PARADO_EN_CRUCE, // Detectó obstáculo, espera autorización
CRUZANDO, // Recibió autorización, cruza
SALIDA, // Publicó salida, vuelve a SEGUIR_LINEA
RETROCESO // Perdió línea, retrocede hasta encontrarla
};

Estado estado = SEGUIR_LINEA;
Estado estadoPrev = SEGUIR_LINEA;

/* ===== Control de timeouts ===== */
unsigned long tEstado = 0; // tiempo de entrada al estado actual
unsigned long tUltimaSolicitud = 0; // tiempo última solicitud enviada
const unsigned long TIMEOUT_CRUZAR_MS = 5000; // timeout para cruzar
const unsigned long TIMEOUT_RETROCESO_MS = 2000; // timeout para evitar quedar atrapado
const unsigned long TIMEOUT_SOLICITUD_MS = 2000; // intervalo para reenviar solicitud

/* ===== Eventos (bitmask) ===== */
enum Evento : uint32_t {
EV_NONE = 0,
EV_OBSTACLE = 1u << 0, // obstáculo detectado por ultrasonido
EV_LINE_LEFT = 1u << 1, // sensor línea izquierda
EV_LINE_RIGHT = 1u << 2, // sensor línea derecha
EV_AUTORIZACION = 1u << 3, // autorización recibida por MQTT
EV_TIMEOUT = 1u << 4 // timeout en estado actual
};

volatile uint32_t eventos_callbacks = EV_NONE; // puesto por callbacks (MQTT, interrupciones)
volatile bool autorizado_flag = false; // marcada al recibir autorización

/* ---------- FUNCIONES ---------- */
void moverMotores(int velIzq, int velDer);
void stopMotores();
void ejecutarSeguirLinea(); // función común para seguir línea
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

// Iniciar en SEGUIR_LINEA con LED VERDE
digitalWrite(CONTROL_LED_VERDE, HIGH);
digitalWrite(CONTROL_LED_ROJO, LOW);
Serial.println("[SETUP] Iniciando en estado SEGUIR_LINEA");
}

/* ---------- LOOP ---------- */
void loop() {
mqtt.loop();

// 1) Detectar eventos
uint32_t ev = detectarEventos();

// 2) Determinar siguiente estado
Estado siguiente = determinarSiguienteEstado(estado, ev);
if (siguiente != estado) {
estadoPrev = estado;
estado = siguiente;
tEstado = millis(); // Reinicia timer para el nuevo estado
Serial.printf("[FSM] %d -> %d (ev=0x%02X, t=%lu)\n",
(int)estadoPrev, (int)estado, ev, millis());
}

// 3) Ejecutar acciones del estado
ejecutarEstado(estado);

delay(2);
}

/* ===== 1) Detectar eventos ===== */
uint32_t detectarEventos() {
uint32_t ev = EV_NONE;

// Eventos desde callbacks MQTT
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
if (distancia < 10.0) { // Cruce detectado
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

// Timeout en CRUZANDO
if (estado == CRUZANDO && (millis() - tEstado) >= TIMEOUT_CRUZAR_MS) {
ev |= EV_TIMEOUT;
Serial.println("[detectarEventos] TIMEOUT cruzando");
}

// Timeout en RETROCESO (evita quedarse atrapado)
if (estado == RETROCESO && (millis() - tEstado) >= TIMEOUT_RETROCESO_MS) {
ev |= EV_TIMEOUT;
Serial.println("[detectarEventos] TIMEOUT retroceso");
}

return ev;
}

/* ===== 2) Determinar siguiente estado ===== */
Estado determinarSiguienteEstado(Estado s, uint32_t ev) {
switch (s) {
case SEGUIR_LINEA:
// Si detecta obstáculo, se detiene y espera autorización
if (ev & EV_OBSTACLE) {
return PARADO_EN_CRUCE;
}
// Si pierde ambos sensores, comienza retroceso
else if ( !(ev & EV_LINE_LEFT) && !(ev & EV_LINE_RIGHT) ) {
return RETROCESO;
}
return SEGUIR_LINEA;

case PARADO_EN_CRUCE:
// Recibe autorización y comienza a cruzar
if (ev & EV_AUTORIZACION) {
return CRUZANDO;
}
return PARADO_EN_CRUCE;

case CRUZANDO:
// Timeout durante el cruce
if (ev & EV_TIMEOUT) {
return SALIDA;
}
return CRUZANDO;

case RETROCESO:
// Vuelve a SEGUIR_LINEA si detecta línea
if ( (ev & EV_LINE_LEFT) || (ev & EV_LINE_RIGHT) ) {
return SEGUIR_LINEA;
}
// O si timeout, también a SEGUIR_LINEA
if (ev & EV_TIMEOUT) {
return SEGUIR_LINEA;
}
return RETROCESO;

case SALIDA:
// Vuelve a SEGUIR_LINEA inmediatamente
return SEGUIR_LINEA;
}
return s;
}

/* ===== 3) Ejecutar estado ===== */
void ejecutarEstado(Estado s) {
switch (s) {
case SEGUIR_LINEA:
digitalWrite(CONTROL_LED_VERDE, HIGH);
digitalWrite(CONTROL_LED_ROJO, LOW);
ejecutarSeguirLinea();
break;

case PARADO_EN_CRUCE:
digitalWrite(CONTROL_LED_ROJO, HIGH);
digitalWrite(CONTROL_LED_VERDE, LOW);
stopMotores();
// Envía solicitud al entrar en estado o cada TIMEOUT_SOLICITUD_MS
if (estadoPrev == SEGUIR_LINEA || (millis() - tUltimaSolicitud) >= TIMEOUT_SOLICITUD_MS) {
enviarSolicitud();
tUltimaSolicitud = millis();
Serial.println("[PARADO_EN_CRUCE] Solicitud enviada");
}
break;

case CRUZANDO:
digitalWrite(CONTROL_LED_VERDE, HIGH);
digitalWrite(CONTROL_LED_ROJO, LOW);
ejecutarSeguirLinea();
break;

case RETROCESO:
digitalWrite(CONTROL_LED_ROJO, HIGH);
digitalWrite(CONTROL_LED_VERDE, LOW);
moverMotores(-VELOCIDAD, -VELOCIDAD);
break;

case SALIDA:
digitalWrite(CONTROL_LED_ROJO, HIGH);
digitalWrite(CONTROL_LED_VERDE, LOW);
if (estadoPrev == CRUZANDO) {
String topic = "vehiculo/veh01/salida";
String payload = "{\"salida\": true, \"timestamp\": " + String(millis()) + "}";
mqtt.publish(topic.c_str(), payload.c_str());
Serial.println("[SALIDA] Cruce completado");
}
break;
}
}

/* ---------- FUNCIONES AUXILIARES ---------- */

void ejecutarSeguirLinea() {
// Lógica común de seguimiento de línea
if (last_izq && last_der) {
moverMotores(VELOCIDAD, VELOCIDAD); // Adelante recto
}
else if (!last_izq && last_der) {
moverMotores(VELOCIDAD, 0); // Gira izquierda
}
else if (last_izq && !last_der) {
moverMotores(0, VELOCIDAD); // Gira derecha
}
else {
stopMotores(); // Línea no detectada
}
}

void enviarSolicitud() {
String topic = "vehiculo/veh01/solicitud";
String payload = "{\"solicitud\": true, \"timestamp\": " + String(millis()) + ", \"estado\": \"PARADO_EN_CRUCE\"}";
mqtt.publish(topic.c_str(), payload.c_str());
Serial.println("[SOLICITUD] Enviada");
}

void stopMotores() {
moverMotores(0, 0);
}

void moverMotores(int velIzq, int velDer) {
// Motor izquierdo
if (velIzq > 0) {
ledcWrite(IN1, velIzq);
ledcWrite(IN2, 0);
}
else if (velIzq < 0) {
ledcWrite(IN1, 0);
ledcWrite(IN2, abs(velIzq));
}
else {
ledcWrite(IN1, 0);
ledcWrite(IN2, 0);
}

// Motor derecho
if (velDer > 0) {
ledcWrite(IN3, velDer);
ledcWrite(IN4, 0);
}
else if (velDer < 0) {
ledcWrite(IN3, 0);
ledcWrite(IN4, abs(velDer));
}
else {
ledcWrite(IN3, 0);
ledcWrite(IN4, 0);
}
}