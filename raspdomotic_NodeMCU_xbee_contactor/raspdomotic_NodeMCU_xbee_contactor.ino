#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Configura los pines para la comunicación con el XBee
SoftwareSerial XBee(3, 2); // RX en pin 3, TX en pin 2

// Configuración WiFi
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";

// Configuración MQTT
const char* mqtt_server = "raspi.local";
const char* mqtt_topic = "xbee/contactor";

// Tiempo máximo de espera en milisegundos (10 segundos)
const unsigned long timeout = 10000;
unsigned long startTime;

WiFiClient espClient;
PubSubClient client(espClient);

// Variable para almacenar el estado anterior del byte 27
bool contactor_abierto_anterior = false;

// Tamaño de la trama esperada (28 bytes en este caso)
const int TRAMA_SIZE = 28;
byte trama[TRAMA_SIZE];
int tramaIndex = 0; // Cambiar nombre para evitar conflicto
bool receiving = false;

void setup_wifi() {
  delay(10);
  // Conexión al WiFi
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Reconectando al WiFi...");
    WiFi.begin(ssid, password);
    unsigned long startReconnectTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && millis() - startReconnectTime < timeout) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No se pudo conectar al WiFi. Reintentando...");
    } else {
      Serial.println("WiFi conectado");
      Serial.println(WiFi.localIP());
    }
  }
}

void reconnectMQTT() {
  // Reintentar conexión si se pierde
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentarlo de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  // Inicia la comunicación serial con el monitor serial del Arduino IDE
  Serial.begin(9600);
  
  // Inicia la comunicación serial con el XBee
  XBee.begin(9600);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  // Revisar la conexión WiFi y reconectar si es necesario
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  // Verifica la conexión al broker MQTT
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Verifica si hay datos disponibles en el puerto serial del XBee
  while (XBee.available()) {
    byte inByte = XBee.read();
    
    // Si encontramos el byte de inicio de trama (0x7E)
    if (inByte == 0x7E) {
      receiving = true;
      tramaIndex = 0;
      trama[tramaIndex++] = inByte; // Añadir 0x7E al inicio de la trama
    } else if (receiving) {
      trama[tramaIndex++] = inByte;
      
      // Si se ha completado la trama, procesarla
      if (tramaIndex == TRAMA_SIZE) {
        receiving = false;
        processTrama();
      }
    }
  }
}

void processTrama() {
  // Imprimir la trama completa para verificación
  for (int i = 0; i < TRAMA_SIZE; i++) {
    Serial.print(trama[i], HEX);
    Serial.print(",");
  }
  Serial.println();

  // Revisar el byte 27 (posición 26 en el arreglo)
  bool contactor_abierto_actual = (trama[26] == 0x01);

  // Si el estado cambia de cerrado a abierto, enviar mensaje MQTT de "abierto"
  if (contactor_abierto_actual && !contactor_abierto_anterior) {
    Serial.println("Contactor abierto, enviando mensaje MQTT");
    client.publish(mqtt_topic, "ABIERTO");
  }
  // Si el estado cambia de abierto a cerrado, enviar mensaje MQTT de "cerrado"
  else if (!contactor_abierto_actual && contactor_abierto_anterior) {
    Serial.println("Contactor cerrado, enviando mensaje MQTT");
    client.publish(mqtt_topic, "CERRADO");
  }

  // Actualizar el estado anterior
  contactor_abierto_anterior = contactor_abierto_actual;
}
