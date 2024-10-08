const int contactorPin = 7;  // Pin de entrada en el Arduino
int lastState = HIGH;

void setup() {
  Serial.begin(115200);  // Comunicación Serial con el ESP8266
  pinMode(contactorPin, INPUT_PULLUP);  // Configura el pin del contactor como entrada
}

void loop() {
  int currentState = digitalRead(contactorPin);

  if (currentState != lastState) {
    if (currentState == LOW) {
      Serial.println("CERRADO");
    } else {
      Serial.println("ABIERTO");
    }
    lastState = currentState;
  }

  delay(100);  // Pequeño retardo para evitar lecturas erráticas
}
