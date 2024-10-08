#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Configuración Wi-Fi
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";

// Configuración del Broker MQTT
const char* mqtt_server = "raspi.local";
const char* mqtt_topic = "sensores/contactor1";

WiFiClient espClient;
PubSubClient client(espClient);

// Declarar variables para gestión de sueño
bool dataReceived = false;

void setup() {
  Serial.begin(9600);  // Configura la comunicación serial con el Arduino UNO

  // Conectar a la red Wi-Fi
  setup_wifi();

  // Configuración del cliente MQTT
  client.setServer(mqtt_server, 1883);

  // Configurar el modo de ahorro de energía en Light Sleep
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
}

void loop() {
  // Comprobar si el cliente MQTT está conectado
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer el estado del contactor desde el puerto serial
  if (Serial.available()) {
    String contactorState = Serial.readStringUntil('\n');
    contactorState.trim();  // Elimina espacios o saltos de línea adicionales

    // Marcar que se ha recibido un dato
    dataReceived = true;

    // Publicar el estado recibido al broker MQTT
    if (contactorState == "CERRADO") {
      Serial.println("Contactor CERRADO");
      client.publish(mqtt_topic, "CERRADO");  // Publica mensaje MQTT
    } else if (contactorState == "ABIERTO") {
      Serial.println("Contactor ABIERTO");
      client.publish(mqtt_topic, "ABIERTO");  // Publica mensaje MQTT
    }
  }

  // Si no se recibe ningún dato, activar el modo de bajo consumo
  if (!dataReceived) {
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP);  // Mantener el modo Light Sleep activo
    Serial.println("Entrando en modo de bajo consumo");
    delay(100);  // Ajusta el retardo según sea necesario
  }

  // Reiniciar la bandera para la siguiente iteración
  dataReceived = false;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Conectado a Wi-Fi");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al Broker MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado");
    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}
