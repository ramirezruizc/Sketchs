// Define DEBUG si quieres habilitar la depuración, comenta para deshabilitar
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT2(x, y) Serial.print(x, y)  // Para manejar el segundo argumento (ej. HEX)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT2(x, y)
#endif

//#include <SoftwareSerial.h> Mas eficiente AltSoftSerial
#include <AltSoftSerial.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>

// Configura SoftwareSerial para la comunicación con el ESP-01 (pines 8 y 9)
//SoftwareSerial espSerial(8, 9); // RX en pin 8, TX en pin 9
AltSoftSerial espSerial; // RX en pin 8, TX en pin 9 por defecto

// Configuración WiFi
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";

// Configuración MQTT
const char* mqtt_server = "192.168.1.4";
const char* mqtt_topic = "xbee/contactor";

WiFiEspClient espClient;
PubSubClient client(espClient);

// Variable para almacenar el estado anterior del byte 21
bool contactor_abierto_anterior = false;

// Tamaño fijo de la trama
const int TRAMA_SIZE = 22;
byte trama[TRAMA_SIZE];
int tramaIndex = 0;
bool receiving = false;

void setup() {
    // Inicia la comunicación serial para el XBee (Hardware Serial)
    Serial.begin(9600); // Serial hardware (RX/TX) para el XBee
    DEBUG_PRINTLN("Iniciando comunicación con XBee...");

    // Inicializa la comunicación serial con el ESP-01 usando SoftwareSerial
    espSerial.begin(9600);
    WiFi.init(&espSerial);

    if (WiFi.status() == WL_NO_SHIELD) {
        DEBUG_PRINTLN("No se detecta el esp01 WiFi");
        while (true); // Detener si no se detecta el módulo WiFi
    }

    setup_wifi();
    client.setServer(mqtt_server, 1883);
}

void setup_wifi() {
    delay(10);
    DEBUG_PRINTLN();
    DEBUG_PRINT("Conectando a ");
    DEBUG_PRINTLN(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DEBUG_PRINT(".");
    }

    DEBUG_PRINTLN();
    DEBUG_PRINTLN("WiFi conectado");
    DEBUG_PRINTLN("Dirección IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Reconectando al WiFi...");
    WiFi.begin(ssid, password);
    delay(500);
  }
}

void reconnectMQTT() {
    while (!client.connected()) {
        DEBUG_PRINT("Intentando conectar al broker MQTT...");
        if (client.connect("Xbee_ESP01_Client")) {
            DEBUG_PRINTLN("Conectado");
        } else {
            DEBUG_PRINT("Fallo, rc=");
            DEBUG_PRINT(client.state());
            DEBUG_PRINTLN(" Intentando de nuevo en 5 segundos");
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

  // Verifica si hay datos disponibles en el puerto serial del XBee
  if (Serial.available()) {
    byte inByte = Serial.read();
    tratarTrama(inByte);
  }
}

void tratarTrama(byte inByte) {
    // Si encontramos el byte de inicio de trama (0x7E)
    if (inByte == 0x7E) {
        receiving = true;
        tramaIndex = 0;
        trama[tramaIndex++] = inByte; // Añadir 0x7E al inicio de la nueva trama
    } else if (receiving) {
        trama[tramaIndex++] = inByte;
        
        // Verificar que hemos recibido exactamente 22 bytes
        if (tramaIndex == TRAMA_SIZE) {
            processTrama(); // Procesar trama completa de 22 bytes
            receiving = false; // Reiniciar recepción
        } else if (tramaIndex > TRAMA_SIZE) {
            receiving = false; // Si excedemos 22 bytes, descartar la trama
        }
    }
}

void processTrama() {
    // Imprimir la trama completa para verificación
    DEBUG_PRINT("Trama completa recibida: ");
    for (int i = 0; i < tramaIndex; i++) {
        DEBUG_PRINT2(trama[i], HEX);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();

    // Revisar el byte 21 (posición 20 en el arreglo)
    bool contactor_abierto_actual = (trama[20] == 0x01);

    // Si el estado cambia de cerrado a abierto, enviar mensaje MQTT de "abierto"
    if (contactor_abierto_actual && !contactor_abierto_anterior) {
        DEBUG_PRINTLN("Contactor abierto, enviando mensaje MQTT");
        client.publish(mqtt_topic, "ABIERTO");
    }
    // Si el estado cambia de abierto a cerrado, enviar mensaje MQTT de "cerrado"
    else if (!contactor_abierto_actual && contactor_abierto_anterior) {
        DEBUG_PRINTLN("Contactor cerrado, enviando mensaje MQTT");
        client.publish(mqtt_topic, "CERRADO");
    }

    // Actualizar el estado anterior
    contactor_abierto_anterior = contactor_abierto_actual;
}