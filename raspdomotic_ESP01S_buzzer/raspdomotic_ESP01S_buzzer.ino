#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Configuración WiFi
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";

// Configuración del broker MQTT
const char* mqtt_server = "192.168.1.4";  // Dirección IP o URL del broker MQTT
const char* topic_buzzer = "esp01s/buzzer";  // Topic del LED, ya que simplificamos

// Pines
const int ledPin = 2;  // GPIO2 del ESP-01S

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.begin(115200);
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
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Mensaje recibido en topic: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  Serial.println(message);

  // Control del LED en lugar del buzzer
  if (String(topic) == topic_buzzer) {
    if (message == "on") {
      Serial.println("Encendiendo LED");
      digitalWrite(ledPin, HIGH);  // Encender LED (GPIO2 en LOW)
    } else if (message == "off") {
      Serial.println("Apagando LED");
      digitalWrite(ledPin, LOW);  // Apagar LED (GPIO2 en HIGH)
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("LED_ESP01S_Client")) {
      Serial.println("Conectado al broker MQTT");
      client.subscribe(topic_buzzer);
    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);   // Configurar IO2 como salida
  digitalWrite(ledPin, LOW); // Apagar el LED
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
