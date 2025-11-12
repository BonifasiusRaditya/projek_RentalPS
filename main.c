#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======== KONFIGURASI WIFI DAN MQTT ========
const char* ssid = "Wokwi-GUEST";  
const char* password = "";          
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883; // Gunakan 1883 (non-SSL) untuk Wokwi

const char* publishTopic = "Tenz/UPN/coffeeShop/sendData";
const char* subscribeTopic = "Tenz/UPN/coffeeShop/getData";

// ======== STRUKTUR DATA ========
struct PublishData {
  String mac;
  bool state;
};

struct SubscribeData {
  String mac;
  bool state;
  int duration;
};

PublishData sendData;
SubscribeData receiveData;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

TaskHandle_t TaskMQTTHandle = NULL;
SemaphoreHandle_t dataMutex;

volatile bool newDataReceived = false;
volatile unsigned long timerStartTime = 0;
volatile bool timerActive = false;

// ======== PIN PERANGKAT ========
const int PIN_HDMI = 33;  // ganti sesuai pin kamu

// ======== DEKLARASI FUNGSI ========
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishData();
void processReceivedData();
void TaskMQTT(void* parameter);

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_HDMI, OUTPUT);
  digitalWrite(PIN_HDMI, LOW);

  sendData.mac = WiFi.macAddress();
  sendData.state = false;

  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Error creating mutex!");
    while (1);
  }

  xTaskCreatePinnedToCore(TaskMQTT, "MQTTTask", 8192, NULL, 2, &TaskMQTTHandle, 1);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ======== KONEKSI WIFI ========
void setupWiFi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
}

// ======== KONEKSI MQTT ========
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    String clientId = "ESP32Client-" + String(WiFi.macAddress());
    Serial.print("Connecting to MQTT... ");
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected!");
      mqttClient.subscribe(subscribeTopic);
      Serial.printf("Subscribed to: %s\n", subscribeTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// ======== CALLBACK MQTT ========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  Serial.print("Payload received: ");
  Serial.println(message);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    receiveData.mac = doc["mac"].as<String>();
    receiveData.state = doc["state"].as<bool>();
    receiveData.duration = doc["duration"].as<int>();

    Serial.println("Data received:");
    Serial.println("MAC: " + receiveData.mac);
    Serial.print("State: "); Serial.println(receiveData.state);
    Serial.print("Duration: "); Serial.println(receiveData.duration);

    newDataReceived = true;
    xSemaphoreGive(dataMutex);
  }
}

// ======== PUBLISH DATA ========
void publishData() {
  if (!mqttClient.connected()) return;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    JsonDocument doc;
    doc["mac"] = sendData.mac;
    doc["state"] = sendData.state;

    String jsonString;
    serializeJson(doc, jsonString);
    xSemaphoreGive(dataMutex);

    if (mqttClient.publish(publishTopic, jsonString.c_str())) {
      Serial.println("Data published successfully:");
      Serial.println(jsonString);
    } else {
      Serial.println("Publish failed!");
    }
  }
}

// ======== PROSES DATA DARI SUBSCRIBE ========
void processReceivedData() {
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    if (receiveData.mac == WiFi.macAddress()) {
      Serial.println("Data ditujukan untuk ESP ini!");

      digitalWrite(PIN_HDMI, receiveData.state ? HIGH : LOW);
      sendData.state = receiveData.state;

      // Jika ada durasi
      if (receiveData.duration > 0) {
        timerStartTime = millis();
        timerActive = true;
        int duration = receiveData.duration * 1000;

        Serial.printf("Timer started for %d ms\n", duration);

        xSemaphoreGive(dataMutex);
        vTaskDelay(duration / portTICK_PERIOD_MS);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          digitalWrite(PIN_HDMI, LOW);
          sendData.state = false;
          timerActive = false;

          Serial.println("Timer selesai, perangkat dimatikan otomatis");
          xSemaphoreGive(dataMutex);
          publishData();
        }
      } else {
        publishData();
        xSemaphoreGive(dataMutex);
      }

    } else {
      Serial.println("Data bukan untuk ESP ini, diabaikan");
      xSemaphoreGive(dataMutex);
    }

    newDataReceived = false;
  }
}

// ======== TASK MQTT ========
void TaskMQTT(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
      setupWiFi();
    }

    if (!mqttClient.connected()) reconnectMQTT();

    mqttClient.loop();
    if (newDataReceived) processReceivedData();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
