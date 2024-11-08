#include <PubSubClient.h>
#include <WiFi.h>
#include <Arduino.h>

#define buttonPin 32  // Pino da botoeira no ESP32

// Cria as instâncias (objetos) das classes
WiFiClient espClient;
PubSubClient client(espClient);

const int motorPin1 = 18;  // IN3 da ponte H
const int motorPin2 = 19;  // IN4 da ponte H

// Variáveis para o controle do motor
int motorSpeed = 460;                          // Velocidade fixa do motor (em microssegundos)
int motorRunning = 0;                          // Estado do motor
unsigned long buttonPressTime = 0;             // Tempo em que o motor foi ligado
unsigned long motorStartTime = 0;              // Tempo em que o motor foi ligado
unsigned long tempoLigado = 0;                 // Duração do funcionamento do motor
bool lastButtonState = HIGH;                   // Estado anterior do botão

/**
 * Função que configura e conecta no WiFI
 */
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

/**
 * Função que configura e conecta no MQTT
 */
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

/**
 * Função que configura os tópicos do MQTT
 */
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

/**
 * Função callback para atender as requisições do MQTT
 */
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
    motorStartTime = millis();  // Marca o tempo de início do motor
    Serial.println("Motor ligado via MQTT");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  setupWiFi();
  setupMqtt();
  setupTopicsMqtt();
}

void loop() {
  client.loop();

  if (motorRunning) {
    // Liga o motor
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

    // Verifica se o tempo de funcionamento acabou
    if (millis() - motorStartTime >= tempoLigado) {
      motorRunning = 0;
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println("Motor desligado");
    }
  }
}
