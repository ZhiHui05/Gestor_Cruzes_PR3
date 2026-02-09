#include <WiFi.h>
#include <PicoMQTT.h>

const char* AP_SSID = "CRUCE_MQTT";
const char* AP_PASS = "12345678";

PicoMQTT::Server mqtt;

/* ===== Estado del cruce ===== */
enum EstadoCruce { CRUCE_LIBRE, CRUCE_OCUPADO };
EstadoCruce estado = CRUCE_LIBRE;

String vehiculoActual = "";
bool solicitudRecibida = false;
bool salidaRecibida = false;
String vehiculoEvento = "";

unsigned long tOcupado = 0;
const unsigned long TIMEOUT_CRUCE = 8000;


//Setup Gestor
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("AP activo. IP: ");
  Serial.println(WiFi.softAPIP());

  // ===== SUSCRIPCIONES MQTT =====

  // Solicitud de paso
  mqtt.subscribe("vehiculo/+/solicitud",
    [](const char* topic, const void*, size_t) {

      String t = topic;
      int i1 = t.indexOf('/');
      int i2 = t.indexOf('/', i1 + 1);

      vehiculoEvento = t.substring(i1 + 1, i2);
      solicitudRecibida = true;
    }
  );

  // Salida del cruce
  mqtt.subscribe("vehiculo/+/salida",
    [](const char* topic, const void*, size_t) {

      String t = topic;
      int i1 = t.indexOf('/');
      int i2 = t.indexOf('/', i1 + 1);

      vehiculoEvento = t.substring(i1 + 1, i2);
      salidaRecibida = true;
    }
  );

  // ===== ARRANCAR BROKER =====
  mqtt.begin();
  Serial.println("Broker MQTT iniciado");
}


//FSM del Gestor
void loop() {
  mqtt.loop();

  switch (estado) {

    case CRUCE_LIBRE:
      if (solicitudRecibida) {
        vehiculoActual = vehiculoEvento;
        solicitudRecibida = false;

        String topic = "gestor/" + vehiculoActual + "/autorizacion";
        mqtt.publish(topic.c_str(), "{ \"ok\": true }");

        Serial.println("Autorizado: " + vehiculoActual);

        estado = CRUCE_OCUPADO;
        tOcupado = millis();
      }
      break;

    case CRUCE_OCUPADO:
      if (salidaRecibida && vehiculoEvento == vehiculoActual) {
        salidaRecibida = false;
        vehiculoActual = "";
        estado = CRUCE_LIBRE;

        Serial.println("Cruce liberado");
      }

      // Timeout de seguridad
      if (millis() - tOcupado > TIMEOUT_CRUCE) {
        Serial.println("Timeout -> cruce liberado");
        vehiculoActual = "";
        estado = CRUCE_LIBRE;
      }
      break;
  }
}

