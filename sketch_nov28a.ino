#include <PubSubClient.h>
#include <WiFi.h>
#include <Arduino.h>

// Pinos do motor
#define buttonPin 32  // Pino da botoeira no ESP32
const int motorPin1 = 18;  // IN3 da ponte H
const int motorPin2 = 19;  // IN4 da ponte H

// Pinos dos sensores
const int presenceSensorPin = 27; // Pino do sensor de presença E18
const int pwSensorPin1 = 32;      // Pino do primeiro sensor (baixa)
const int pwSensorPin2 = 33;      // Pino do segundo sensor (média)
const int pwSensorPin3 = 34;      // Pino do terceiro sensor (alta)

// Instâncias WiFi e MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Variáveis para o motor
int motorSpeed = 460;
int motorRunning = 0;
unsigned long buttonPressTime = 0;
unsigned long motorStartTime = 0;
unsigned long tempoLigado = 0;
bool lastButtonState = HIGH;

// Funções WiFi e MQTT
void setupWiFi() {
  const char *nomeRedeWiFi = "paitech";
  const char *password = "123456789";

  WiFi.begin(nomeRedeWiFi, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando WiFi...");
  }
  Serial.println("Conectado à rede WiFi.");
}

void setupMqtt() {
  const char *enderecoServidorMqtt = "broker.hivemq.com";
  const int portaServidorMqtt = 1883;

  client.setServer(enderecoServidorMqtt, portaServidorMqtt);
  client.setCallback(callback);

  while (!client.connected()) {
    String clientIdStr = "esp32_" + String(random(0xffff), HEX);
    const char *clientId = clientIdStr.c_str();

    if (client.connect(clientId)) {
      Serial.println("Conectado ao broker MQTT!");
    } else {
      Serial.print("Falha na conexão. Código de erro: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setupTopicsMqtt() {
  const char *mqttTopicoAlimentar = "ALIMENTAR";
  const char *mqttTopicoAlimentar05 = "ALIMENTAR05";
  const char *mqttTopicoAlimentar10 = "ALIMENTAR10";
  const char *mqttTopicoAlimentar15 = "ALIMENTAR15";
  const char *mqttTopicoAlimentar20 = "ALIMENTAR20";

  client.subscribe(mqttTopicoAlimentar);
  client.subscribe(mqttTopicoAlimentar05);
  client.subscribe(mqttTopicoAlimentar10);
  client.subscribe(mqttTopicoAlimentar15);
  client.subscribe(mqttTopicoAlimentar20);
}

void callback(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, "ALIMENTAR") == 0) {
    tempoLigado = 5000;
    motorRunning = 1;
  } else if (strcmp(topic, "ALIMENTAR05") == 0) {
    tempoLigado = 5000;
    motorRunning = 1;
  } else if (strcmp(topic, "ALIMENTAR10") == 0) {
    tempoLigado = 10000;
    motorRunning = 1;
  } else if (strcmp(topic, "ALIMENTAR15") == 0) {
    tempoLigado = 15000;
    motorRunning = 1;
  } else if (strcmp(topic, "ALIMENTAR20") == 0) {
    tempoLigado = 20000;
    motorRunning = 1;
  }

  if (motorRunning) {
    motorStartTime = millis();
    Serial.println("Motor ligado via MQTT");
  }
}

void setup() {
  Serial.begin(115200);

  // Configurações dos pinos
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(presenceSensorPin, INPUT);
  pinMode(pwSensorPin1, INPUT);
  pinMode(pwSensorPin2, INPUT);
  pinMode(pwSensorPin3, INPUT);

  // Inicialização do WiFi e MQTT
  setupWiFi();
  setupMqtt();
  setupTopicsMqtt();
}

void loop() {
  client.loop();

  // Controle do motor
  if (motorRunning) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

    if (millis() - motorStartTime >= tempoLigado) {
      motorRunning = 0;
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println("Motor desligado");
    }
  }

  // Leitura dos sensores
  int presenceState = digitalRead(presenceSensorPin);

  if (presenceState == LOW) {
    Serial.println("Presença detectada");
  } else {
    Serial.println("Nenhuma presença detectada");
  }

  int sensorValue1 = digitalRead(pwSensorPin1);
  int sensorValue2 = digitalRead(pwSensorPin2);
  int sensorValue3 = digitalRead(pwSensorPin3);

  if (sensorValue3 == LOW) {
    Serial.println("Estado: Alta");
  } else if (sensorValue2 == LOW) {
    Serial.println("Estado: Média");
  } else if (sensorValue1 == LOW) {
    Serial.println("Estado: Baixa");
  } else {
    Serial.println("Estado: Nenhuma");
  }

  delay(5000);
}
