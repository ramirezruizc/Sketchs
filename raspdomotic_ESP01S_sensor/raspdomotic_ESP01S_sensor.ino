#include <Adafruit_AHT10.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Define para habilitar o deshabilitar la depuración
//#define DEBUG  // Comentar esta línea para desactivar la depuración

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Configuración de red WiFi
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";

// Configuración del servidor MQTT
const char* mqtt_server = "192.168.1.4";
const int mqtt_port = 1883;

const char* mqtt_topic = "sensor/restart";

WiFiClient espClient;
PubSubClient client(espClient);

// Crear instancia del sensor AHTX0
Adafruit_AHT10 aht;

// Tiempo máximo de espera en milisegundos (10 segundos)
const unsigned long timeout = 10000;
unsigned long startTime;

// Variables para controlar el tiempo
unsigned long previousMillis = 0;
const long interval = 30000; // 30 segundos

// Función de callback para manejar mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  DEBUG_PRINT("Mensaje recibido en el tema [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("]: ");
  DEBUG_PRINTLN(message);

  // Verificar si el mensaje es para reiniciar el ESP
  if (String(topic) == "sensor/restart" && message == "restart") {
    DEBUG_PRINTLN("Recibido comando de reinicio. Reiniciando el ESP...");
    client.publish("restart", "ESP01S restart");
    delay(2000);
    ESP.restart();  // Reiniciar el ESP
  }
}

void setup_wifi() {
  delay(100);
  DEBUG_PRINT("Conectando a ");
  DEBUG_PRINTLN(ssid);

  // Conectar a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("Conectado a la WiFi");
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(WiFi.localIP());
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Reconectando al WiFi...");
    WiFi.begin(ssid, password);
    
    DEBUG_PRINTLN("WiFi conectado");
    DEBUG_PRINTLN(WiFi.localIP());
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    DEBUG_PRINT("Conectando al servidor MQTT...");
    if (client.connect("ESP01S_AHT10_Client")) {
      DEBUG_PRINTLN("conectado");
      client.subscribe(mqtt_topic);
    } else {
      DEBUG_PRINT("fallo, rc=");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" intentando en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializar I2C en GPIO 0 para SDA y GPIO 2 para SCL
  Wire.begin(0, 2);

  startTime = millis();  // Guardar el tiempo de inicio

  // Conectar al WiFi
  setup_wifi();

  // Configurar cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (client.connect("ESP01S_AHT10_Client")) {
    DEBUG_PRINTLN("Conectado al broker MQTT");
    client.subscribe(mqtt_topic);
  }

  // Intentar inicializar el sensor AHTX0
  bool sensorInitialized = false;
  
  while (millis() - startTime < timeout) {
    if (aht.begin()) {
      DEBUG_PRINTLN("Sensor AHT10 o AHT20 encontrado");
      sensorInitialized = true;
      break;  // Salir del bucle si el sensor se inicializa correctamente
    }
    delay(500);  // Esperar 500 ms antes de intentar nuevamente
  }

  if (!sensorInitialized) {
    DEBUG_PRINTLN("No se pudo encontrar el sensor AHT10 o AHT20. ¡Enviando mensaje de error!");
    if (client.connect("ESP01Client")) {
      client.publish("raspdomotic/errors", "Sensor temperatura/humedad no detectado");
    }
  }
}

void loop() {
  // Revisar la conexión WiFi y reconectar si es necesario
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  // Revisar la conexión MQTT y reconectar si es necesario
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Obtener el tiempo actual
  unsigned long currentMillis = millis();

  // Verificar si ha pasado el intervalo de 30 segundos
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Actualizar el último tiempo de ejecución

    // Leer temperatura y humedad del sensor
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);  // Poblar los objetos temp y humidity con datos nuevos

    // Publicar los datos en el servidor MQTT
    String temperatureString = String(temp.temperature);
    String humidityString = String(humidity.relative_humidity);

    String payload = "{\"temperature\": " + temperatureString + ", \"humidity\": " + humidityString + "}";
    client.publish("esp01s/temperatura_humedad", payload.c_str());

    DEBUG_PRINT("Temperatura: ");
    DEBUG_PRINT(temp.temperature);
    DEBUG_PRINT(" °C, Humedad: ");
    DEBUG_PRINT(humidity.relative_humidity);
    DEBUG_PRINTLN(" %");
  }
}