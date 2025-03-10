#include <ESP8266WiFi.h>
#include <PubSubClient.h>  // Biblioteca MQTT

// Define para habilitar o deshabilitar la depuración
//#define DEBUG  // Comentar esta línea para desactivar la depuración

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Configura los pines
#define ACTIVATION_PIN 0  // Usamos GPIO0 del ESP-01S para activar la cámara

// Configuración del Wi-Fi y MQTT
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";
const char* mqtt_server = "192.168.1.4";  // IP o dominio del servidor MQTT
const char* mqtt_topic = "esp01s/camara"; // Tópico para activar la cámara

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  
  // Configura el pin como salida
  pinMode(ACTIVATION_PIN, OUTPUT);
  digitalWrite(ACTIVATION_PIN, HIGH);  // Inicia con la señal baja

  // Conéctate a Wi-Fi
  setup_wifi();

  // Configura el cliente MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
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
  Serial.println("WiFi conectado");
  Serial.println(WiFi.localIP());
}

// Función que se ejecuta cuando llega un mensaje MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tópico: ");
  Serial.println(topic);

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if(String(topic) == mqtt_topic && message == "activar") {
    Serial.println("Activando cámara...");
    
    // Enviar señal HIGH al ESP32-CAM (activar la cámara)
    digitalWrite(ACTIVATION_PIN, LOW);
    delay(1000);  // Mantener la señal por un breve tiempo
    digitalWrite(ACTIVATION_PIN, HIGH);
  }
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Reconectando al WiFi...");
    WiFi.begin(ssid, password);
    
    DEBUG_PRINTLN("WiFi conectado");
    DEBUG_PRINTLN(WiFi.localIP());
  }
}

// Función para reconectar al servidor MQTT si se desconecta
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando al servidor MQTT...");
    if (client.connect("ESP32_CAM_ESP01S_Client")) {
      Serial.println("conectado");
      client.subscribe(mqtt_topic);  // Nos suscribimos al tópico de activación
    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void loop() {
  // Revisar la conexión WiFi y reconectar si es necesario
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
}
