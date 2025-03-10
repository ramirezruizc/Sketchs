#include <WiFi.h>
#include <WebSocketsClient.h>

const char* ssid = "TCNHYLADO";
const char* password = "suriru88";
const char* serverAddress = "192.168.1.4"; // Cambia por tu servidor
const int serverPort = 7000;

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("🔴 Desconectado del servidor WebSocket");
            break;
        case WStype_CONNECTED:
            Serial.println("🟢 Conectado al servidor WebSocket");
            webSocket.sendTXT("STATUS"); // Envía mensaje inicial
            break;
        case WStype_TEXT:
            Serial.printf("📩 Mensaje recibido: %s\n", payload);
            break;
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("🔄 Conectando a WiFi...");
    }

    Serial.println("✅ Conectado a WiFi");

    webSocket.begin(serverAddress, serverPort, "/ws"); // Ruta `/ws`
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop();
}
