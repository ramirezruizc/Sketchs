#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include <ESP_Mail_Client.h>
#include <WebServer.h> // Incluye la biblioteca para el servidor web
#include <FS.h>
#include <SD_MMC.h>
#include <time.h>
#include "driver/rtc_io.h"

// Configuración de la cámara
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//El pin 4 se utiliza en la gestion de la SD en 4-bit
//podemos inicializar la SD en modo 1-bit
//y asi poder gestionar el flash a demanda
#define FLASH_GPIO GPIO_NUM_4 // Pin del flash

// Definir el pin de activación
#define ACTIVATION_PIN GPIO_NUM_13

// Habilitar o deshabilitar la depuración
#define DEBUG_ENABLED 1

// Definir el macro de depuración
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Configuración de red y MQTT
const char* ssid = "TCNHYLADO";
const char* password = "suriru88";
const char* mqtt_server = "192.168.1.4";  // IP del broker Mosquitto
//const char* mqtt_topic = "camara/control";
const char* mqtt_topic = "esp01s/camara";

// Configuración de IP estática
IPAddress staticIP(192, 168, 1, 100); // Cambia a la IP deseada
IPAddress gateway(192, 168, 1, 1);    // IP del gateway (generalmente el router)
IPAddress subnet(255, 255, 255, 0);   // Máscara de subred
IPAddress primaryDNS(192, 168, 1, 1);       // DNS Primario (por ejemplo, Google DNS)

WiFiClient espClient;
PubSubClient client(espClient);

// Crear un mutex global
SemaphoreHandle_t cameraMutex;

const char* ntpServer = "pool.ntp.org";
//const char* ntpServer = "time.google.com";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// Configuración del servidor SMTP
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 587

#define AUTHOR_EMAIL "ramirezruiz.c@gmail.com"
#define AUTHOR_PASSWORD "tqyo letd ydtw ouuu"

// Variables de la sesión SMTP
SMTPSession smtp;
SMTP_Message message;

camera_config_t config;
WebServer server(80); // Inicializa el servidor web en el puerto 80

// Función para inicializar el mutex
void initializeMutex() {
  cameraMutex = xSemaphoreCreateMutex();
  if (cameraMutex == NULL) {
    DEBUG_PRINTLN("Error creando el mutex");
  }
}

// Inicializa la cámara con la configuración dada
void initializeCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    DEBUG_PRINTLN("PSRAM encontrada.");
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2; //Debe ser mayor a 1
    config.grab_mode = CAMERA_GRAB_LATEST; // Siempre usa el frame más reciente
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2; //Debe ser mayor a 1
    config.grab_mode = CAMERA_GRAB_LATEST; // Siempre usa el frame más reciente
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PRINT("Error inicializando la cámara: ");
    DEBUG_PRINTLN(err);
  } else {
    DEBUG_PRINTLN("Cámara inicializada");
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
  /*  s->set_vflip(s, 1); // Voltear verticalmente (180 grados)
    s->set_gain_ctrl(s, 1);                       // auto gain on
    s->set_exposure_ctrl(s, 1);                   // auto exposure on
    s->set_awb_gain(s, 1);                        // Auto White Balance enable (0 or 1)
    s->set_brightness(s, 1);                     // (-2 to 2) - set brightness*/

    // BRILLO (-2 to 2)
    s->set_brightness(s, 0);
    // CONTRASTE (-2 to 2)
    s->set_contrast(s, 0);
    // SATURACIÓN (-2 to 2)
    s->set_saturation(s, 0);
    // EFECTOS ESPECIALES (0 - Sin efecto, 1 - Negativo, 2 - Escala de grises, 3 - Tinte rojo, 4 - Tinte verde, 5 - Tinte azul, 6 - Sepia)
    s->set_special_effect(s, 0);
    //BALANCE DE BLANCOS (0 = Desactivar, 1 = Activar)
    s->set_whitebal(s, 1);
    // GANANCIA AWB (0 = Desactivar, 1 = Activar)
    s->set_awb_gain(s, 1);
    // MODOS WB (0 - Automático, 1 - Soleado, 2 - Nublado, 3 - Oficina, 4 - Hogar)
    s->set_wb_mode(s, 0);
    // CONTROLES DE EXPOSICIÓN (0 = Desactivar, 1 = Activar)
    s->set_exposure_ctrl(s, 1);
    // AEC2 (0 = Desactivar, 1 = Activar)
    s->set_aec2(s, 0);
    // NIVELES DE AE (-2 a 2)
    s->set_ae_level(s, 0);
    // VALORES AEC (0 a 1200)
    s->set_aec_value(s, 300);
    // CONTROLES DE GANANCIA (0 = Desactivar, 1 = Activar)
    s->set_gain_ctrl(s, 1);
    // GANANCIA AGC (0 a 30)
    s->set_agc_gain(s, 0);
    // GANANCIA TECHO (0 a 6)
    s->set_gainceiling(s, (gainceiling_t)0);
    // BPC (0 = Desactivar, 1 = Activar)
    s->set_bpc(s, 0);
    // WPC (0 = Desactivar, 1 = Activar)
    s->set_wpc(s, 1);
    // GMA RAW (0 = Desactivar, 1 = Activar)
    s->set_raw_gma(s, 1);
    // LENC (0 = Desactivar, 1 = Activar)
    s->set_lenc(s, 1);
    // ESPEJO ORIZ (0 = Desactivar, 1 = Activar)
    s->set_hmirror(s, 0);
    // VERT FLIP (0 = Desactivar, 1 = Activar)
    s->set_vflip(s, 1);
    // DCW (0 = Desactivar, 1 = Activar)
    s->set_dcw(s, 1);
    // PATRÓN DE BARRA DE COLOR (0 = Desactivar, 1 = Activar)
    s->set_colorbar(s, 0);
  }
}

// Inicializar la tarjeta SD
void initializeSDCard() {
  if (!SD_MMC.begin()) {
  //if (!SD_MMC.begin("/sdcard", true)) { //Inicializar en 1-bit mode
    DEBUG_PRINTLN("No se pudo montar la tarjeta SD");
  }
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  DEBUG_PRINT("Tamaño de la tarjeta SD: ");
  DEBUG_PRINT(cardSize);
  DEBUG_PRINTLN("MB");
}

void initTime() {
  // Verificar si está conectado al WiFi
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTLN("WiFi no está conectado, no se puede sincronizar NTP.");
    return;  // Evitar entrar en el bucle si no hay conexión WiFi
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;

  // Intentar obtener la hora un número limitado de veces para evitar bloqueo infinito
  int retry = 0;
  const int max_retries = 10;  // Intentar durante 10 segundos

  while (!getLocalTime(&timeinfo) && retry < max_retries) {
    DEBUG_PRINTLN("Esperando sincronización con el servidor NTP...");
    delay(1000);  // Espera de 1 segundo antes de volver a intentar
    retry++;
  }

  if (retry >= max_retries) {
    DEBUG_PRINTLN("Error: No se pudo sincronizar con el servidor NTP.");
  } else {
    DEBUG_PRINTLN("Sincronización NTP completada.");
  }
}

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    DEBUG_PRINTLN("Fallo al obtener la hora");
    return "00-00-00_00-00-00";
  }
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &timeinfo);
  return String(buffer);
}

// Encender el flash
void turnOnFlash() {
  digitalWrite(FLASH_GPIO, HIGH);
}

// Apagar el flash
void turnOffFlash() {
  digitalWrite(FLASH_GPIO, LOW);
}

// Función para encender el LED brevemente
void flashWiFiConnected() {
  turnOnFlash();
  delay(300); // LED encendido brevemente
  turnOffFlash();
}

// Función para parpadeo rápido para conexión MQTT
void flashMQTTConnected() {
  for (int i = 0; i < 2; i++) {
    turnOnFlash();
    delay(100); // LED encendido rápido
    turnOffFlash();
    delay(100); // Pausa rápida
  }
}

// Función para capturar imagen, guardar en SD y enviar correo
void captureSaveAndSendEmail() {
  //Bloqueamos con semaforo
  /*if (xSemaphoreTake(cameraMutex, 0) == pdTRUE) {
    DEBUG_PRINTLN("Camara bloqueda por semaforo");*/

    // Inicializa la cámara antes de capturar la imagen
    //initializeCamera();

    // Encender el flash
    //turnOnFlash();
    
    //delay(500); // Pequeño retardo para permitir que el flash ilumine correctamente

    camera_fb_t * fb = NULL;

    delay(100);

    pinMode(4, OUTPUT);  // Pin 4 para el flash
    digitalWrite(4, HIGH);  // Activa el flash

    delay(500);

    // Capturar imagen con ESP32-CAM
    fb = esp_camera_fb_get();

    // Apagar el flash inmediatamente después de capturar la imagen
    turnOffFlash();

    if (!fb) {
      DEBUG_PRINTLN("Error capturando la imagen");
      //esp_camera_deinit(); // Liberar recursos de la cámara
      esp_camera_fb_return(fb); // Liberar el frame buffer
      //xSemaphoreGive(cameraMutex); // Liberar el mutex
      return;
    } else {
      DEBUG_PRINTLN("Imagen capturada");
    }

    // Obtener la fecha y hora actual
    String currentTime = getFormattedTime();

    initializeSDCard();

    // Crear la carpeta principal si no existe
    if (!SD_MMC.exists("/capturas")) {
        if (SD_MMC.mkdir("/capturas")) {
            DEBUG_PRINTLN("Carpeta /capturas creada exitosamente.");
        } else {
            DEBUG_PRINTLN("Error al crear la carpeta /capturas.");
        }
    }

    // Crear la carpeta por día
    String folderPath = "/capturas/" + currentTime.substring(0, 10); // Carpeta del día
    if (!SD_MMC.exists(folderPath)) {
      DEBUG_PRINTLN("Carpeta del dia no creada. Intentando crearla...");
      if (SD_MMC.mkdir(folderPath)) {
        DEBUG_PRINTLN("Carpeta del dia creada exitosamente.");
      } else {
        DEBUG_PRINTLN("Error al crear la carpeta.");
        return;
      }
    }

    // Generar el nombre del archivo con fecha y hora
    String fileName = folderPath + "/" + currentTime + ".jpg";
    File file = SD_MMC.open(fileName.c_str(), FILE_WRITE);

    // Guardar la imagen en la SD
    if (file) {
      file.write(fb->buf, fb->len);
      DEBUG_PRINTLN("Imagen guardada en: " + fileName);
      file.close();
    } else {
      DEBUG_PRINTLN("Error al guardar la imagen en la tarjeta SD");
    }

    // Configuración del mensaje de correo
    message.sender.name = "Raspdomotic";
    message.sender.email = AUTHOR_EMAIL;
    message.subject = "Captura de la cámara ESP32";
    message.addRecipient("César", "ramirezruiz.c@gmail.com");

    // Limpiar el mensaje anterior
    message.clearAttachments();

    // Adjuntar la imagen al correo
    SMTP_Attachment attachment;
    attachment.descr.filename = "imagen.jpg";
    attachment.descr.mime = "image/jpg";
    attachment.blob.data = fb->buf;
    attachment.blob.size = fb->len;
    message.addAttachment(attachment);

    // Configurar sesión SMTP
    ESP_Mail_Session sessionConfig;
    sessionConfig.server.host_name = SMTP_HOST;
    sessionConfig.server.port = SMTP_PORT;
    sessionConfig.login.email = AUTHOR_EMAIL;
    sessionConfig.login.password = AUTHOR_PASSWORD;
    sessionConfig.login.user_domain = "";

    // Conectar y enviar el correo
    if (!smtp.connect(&sessionConfig)) {
      DEBUG_PRINTLN("Error conectando al servidor SMTP");
      DEBUG_PRINTLN("Motivo del error: " + smtp.errorReason());
      //xSemaphoreGive(cameraMutex); // Liberar el mutex
      smtp.closeSession();
      esp_camera_fb_return(fb); // Liberar el frame buffer
      //esp_camera_deinit(); // Liberar recursos de la cámara
      return;
    }

    if (!MailClient.sendMail(&smtp, &message)) {
      DEBUG_PRINTLN("Error enviando el correo");
    } else {
      DEBUG_PRINTLN("Correo enviado correctamente");
    }

    // Limpiar la sesión SMTP después de cada envío
    smtp.closeSession();

    esp_camera_fb_return(fb); // Liberar el frame buffer
    //esp_camera_deinit(); // Liberar recursos de la cámara

    //xSemaphoreGive(cameraMutex); // Liberar el mutex
    //DEBUG_PRINTLN("Camara desbloqueda del semaforo");
  /*} else {
    DEBUG_PRINTLN("No se pudo adquirir el mutex. Otra operación en curso. Intente de nuevo mas tarde");
  }*/
}

// Callback SMTP
void smtpCallback(SMTP_Status status) {
  DEBUG_PRINT("Estado del envío: ");
  DEBUG_PRINTLN(status.info());
}

// Callback de MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == mqtt_topic && message == "activar") {
    DEBUG_PRINTLN("Capturando y enviando imagen por email");
    captureSaveAndSendEmail();
  }
}

// Función para manejar la captura de imagen bajo demanda
// Captura de imagen bajo demanda
//void handleCapture(void *pvParameters) {
void handleCapture() {
  //Bloqueamos con semaforo
  /*if (xSemaphoreTake(cameraMutex, 0) == pdTRUE) {
    DEBUG_PRINTLN("Camara bloqueda por semaforo");*/
    // Inicializa la cámara antes de capturar la imagen
    //initializeCamera();
  
    if (server.hasArg("flash")) {
      String flashState = server.arg("flash");
      if (flashState == "on") {
        turnOnFlash();
        delay(500); // Esperar a que el flash se encienda
      }
    }

    camera_fb_t * fb = esp_camera_fb_get();
    
    if (!fb) {
      server.send(500, "text/plain", "Error al capturar la imagen");
      turnOffFlash();
      //xSemaphoreGive(cameraMutex); // Liberar el mutex
      esp_camera_fb_return(fb); // Liberar el frame buffer
      //esp_camera_deinit(); // Liberar recursos de la cámara
      return;
    }
    delay(1000); // Esperar a que el flash se encienda
    turnOffFlash();

    // Enviar encabezados HTTP para imagen JPEG
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(fb->len));

    // Enviar la imagen
    WiFiClient client = server.client();
    client.write(fb->buf, fb->len);

    // Finalizar la respuesta HTTP
    server.send(200, ""); // Solo se envía el encabezado

    // Liberar el frame buffer
    esp_camera_fb_return(fb);

    //xSemaphoreGive(cameraMutex); // Liberar el mutex
    //DEBUG_PRINTLN("Camara desbloqueda del semaforo");

    //esp_camera_deinit(); // Liberar recursos de la cámara
  /*} else {
    DEBUG_PRINTLN("No se pudo adquirir el mutex. Otra operación en curso. Intente de nuevo mas tarde");
  }*/
}

// Función para manejar el stream de la cámara
// Función para manejar el stream
void handleStream() {
  //Bloqueamos con semaforo
  /*if (xSemaphoreTake(cameraMutex, 0) == pdTRUE) {
    DEBUG_PRINTLN("Camara bloqueada por semaforo");*/
    // Inicializa la cámara antes de capturar la imagen
    //initializeCamera();

    if (server.hasArg("flash")) {
      String flashState = server.arg("flash");
      if (flashState == "on") {
        turnOnFlash();
        delay(1000); // Esperar a que el flash se encienda
      }
    }

    camera_fb_t * fb = NULL;
    WiFiClient client = server.client();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    while (client.connected()) {
      fb = esp_camera_fb_get();
      if (!fb) {
        DEBUG_PRINTLN("Error capturando la imagen");
        break;
      }

      client.printf("--frame\r\n");
      client.printf("Content-Type: image/jpeg\r\n");
      client.printf("Content-Length: %u\r\n\r\n", fb->len);
      client.write(fb->buf, fb->len);
      client.printf("\r\n");

      esp_camera_fb_return(fb);
      delay(100);
    }
    turnOffFlash();

    // Liberar el frame buffer
    esp_camera_fb_return(fb);

//    xSemaphoreGive(cameraMutex); // Liberar el mutex
//    DEBUG_PRINTLN("Camara desbloqueda del semaforo");

    //esp_camera_deinit(); // Liberar recursos de la cámara
/*  } else {
    DEBUG_PRINTLN("No se pudo adquirir el mutex. Otra operación en curso. Intente de nuevo mas tarde");
  }*/
}

/*// Función para manejar el streaming de la cámara
void handleStreamTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(cameraMutex, portMAX_DELAY)) {
      handleStream();
      xSemaphoreGive(cameraMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay para evitar ocupación excesiva del CPU
  }
}*/

void handleFlashControl() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      turnOnFlash();
      server.send(200, "text/plain", "Flash activado");
    } else if (state == "off") {
      turnOffFlash();
      server.send(200, "text/plain", "Flash desactivado");
    } else {
      server.send(400, "text/plain", "Estado no válido");
    }
  } else {
    server.send(400, "text/plain", "Parámetro 'state' faltante");
  }
}

void setup_Wifi() {
  delay(100);
  DEBUG_PRINT("Conectando a ");
  DEBUG_PRINTLN(ssid);

  // Conectar a la red WiFi
  WiFi.config(staticIP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("Conectado a la WiFi");
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(WiFi.localIP());

  flashWiFiConnected(); // Confirmación visual
}

void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("Reconectando al WiFi...");
    // Conectar a la red WiFi
    WiFi.config(staticIP, gateway, subnet, primaryDNS);
    WiFi.begin(ssid, password);
    
    DEBUG_PRINTLN("WiFi conectado");
    DEBUG_PRINTLN(WiFi.localIP());

    flashWiFiConnected(); // Confirmación visual
  }
}

void reconnectMQTT() {
  // Bucle hasta que se reconecte
  while (!client.connected()) {
    DEBUG_PRINTLN("Intentando reconectar al broker MQTT...");
    
    // Intenta conectar
    if (client.connect("ESP32_CAM_Client")) {
      DEBUG_PRINTLN("Conectado al broker MQTT");
      client.subscribe(mqtt_topic);

      //flashMQTTConnected()
    } else {
      DEBUG_PRINT("Error de conexión, rc=");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" Intentando de nuevo en 5 segundos");
      
      // Espera 5 segundos antes de intentar reconectar
      delay(5000);
    }
  }
}

void setup() {
/* 
**********************************************************************************************
**********************************************************************************************
USO NORMAL DE LA CAMARA SIN HACER DEEP_SLEEP
**********************************************************************************************
**********************************************************************************************
*/
/*
  //Inicializamos mutex (semaforo)
  //initializeMutex();

  //xSemaphoreGive(cameraMutex); // Inicialmente disponible

  /* // Crear tareas
  xTaskCreatePinnedToCore(captureSaveAndSendEmailTask, "CaptureSaveAndSendEmailTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(handleCaptureTask, "HandleCaptureTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(handleStreamTask, "HandleStreamTask", 4096, NULL, 1, NULL, 1);* /
  // Inicializar el pin del flash como salida
  pinMode(FLASH_GPIO, OUTPUT);
  turnOffFlash(); // Asegurarse que el LED esté apagado al inicio

  Serial.begin(115200);

  // Inicializa la cámara antes de capturar la imagen
  initializeCamera();

  initializeSDCard();

  setup_Wifi();
  
  // Conectar al broker MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //client.setKeepAlive(60); // Ajusta el intervalo de keep-alive a 60 segundos

  if (client.connect("ESP32_CAM_Client")) {
    DEBUG_PRINTLN("Conectado al broker MQTT");
    flashMQTTConnected(); // Confirmación visual con parpadeo
    client.subscribe(mqtt_topic);
  }

  initTime();

  // Configurar el servidor web
  server.on("/", HTTP_GET, [](){
  server.send(200, "text/html", "<h1>ESP32-CAM</h1>"
                                "<p><a href=\"/stream\">Ver Stream</a></p>"
                                "<p><a href=\"/capture?flash=on\">Captura Imagen con Flash</a></p>"
                                "<p><a href=\"/capture?flash=off\">Captura Imagen sin Flash</a></p>"
                                "<p><a href=\"/flash?state=on\">Activar Flash</a></p>"
                                "<p><a href=\"/flash?state=off\">Desactivar Flash</a></p>");
  });

  /*server.on("/", HTTP_GET, [](){
    server.send(200, "text/html", "<h1>ESP32-CAM</h1><p><a href=\"/stream\">Ver Stream</a></p><p><a href=\"/capture\">Captura Imagen</a></p>");
  });* /

  server.on("/stream", HTTP_GET, handleStream);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/flash", HTTP_GET, handleFlashControl);

  server.begin();

  // Comprobar si el ESP32-CAM fue despertado por la señal externa
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    // Capturar la imagen y enviarla por correo
    captureSaveAndSendEmail();
  }

  // Configurar el pin 13 para activar el despertador por señal externa
  esp_sleep_enable_ext0_wakeup(ACTIVATION_PIN, 1);  // Despertar con una señal HIGH en el pin 13

  // Poner el ESP32 en modo de suspensión profunda
  delay(1000);
  esp_deep_sleep_start();
*/
/* 
**********************************************************************************************
**********************************************************************************************
FIN USO NORMAL DE LA CAMARA SIN HACER DEEP_SLEEP
**********************************************************************************************
**********************************************************************************************
*/
/* 
**********************************************************************************************
**********************************************************************************************
USO NORMAL DE LA CAMARA CON DEEP_SLEEP
**********************************************************************************************
**********************************************************************************************
*/
  pinMode(FLASH_GPIO, OUTPUT);
  turnOffFlash(); // Asegurarse que el LED esté apagado al inicio

  Serial.begin(115200);

  // Comprobar si el ESP32-CAM fue despertado por la señal externa
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    //initializeSDCard();

    rtc_gpio_hold_dis(FLASH_GPIO);

    initializeCamera();

    setup_Wifi();

    initTime();
    
    captureSaveAndSendEmail();

    SD_MMC.end();
  }

  //pinMode(ACTIVATION_PIN, INPUT_PULLUP);  // Configura el pin 13 con pull-up
  pinMode(FLASH_GPIO, OUTPUT);
  turnOffFlash(); // Asegurarse que el LED esté apagado
  
  // Mantén el estado del GPIO 4 en deep sleep
  rtc_gpio_hold_en(FLASH_GPIO);

  // Configurar el pin 13 para activar el despertador por señal externa
  esp_sleep_enable_ext0_wakeup(ACTIVATION_PIN, LOW);

  // Poner el ESP32 en modo de suspensión profunda
  DEBUG_PRINTLN("Entrando en modo Deep Sleep...");
  delay(100);  // Espera un momento antes de dormir
    
  // Entrar en deep sleep
  esp_deep_sleep_start();
}

/*void loop() {
  // Revisar la conexión WiFi y reconectar si es necesario
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  server.handleClient(); // Maneja las solicitudes HTTP
}*/

void loop() {
 
}