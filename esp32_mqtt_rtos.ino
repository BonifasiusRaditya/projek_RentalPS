#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "";
const char* password = ""; // Nantian diisi aja

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 8883;

const char* publishTopic = "Tenz/UPN/coffeeShop/sendData";
const char* subscribeTopic = "Tenz/UPN/coffeeShop/getData";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

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
String deviceMAC;

TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;

SemaphoreHandle_t dataMutex;

volatile bool newDataReceived = false;
volatile unsigned long timerStartTime = 0;
volatile bool timerActive = false;

const int PIN_HDMI = 2; // sesuain ama pin

void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void TaskMQTT(void* parameter);
void publishData();
void processReceivedData();
void setupWiFi();

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
  mqttClient.setBufferSize(512); // size for JSON
  
  dataMutex = xSemaphoreCreateMutex();
  
  if (dataMutex == NULL) {
    Serial.println("Error creating mutex!");
    while(1);
  }

  xTaskCreatePinnedToCore(TaskMQTT, "MQTTTask", 8192, NULL, 2, &Task2Handle, 1);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Reconnect to MQTT broker
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("mqtt connected!");
      if (mqttClient.subscribe(subscribeTopic)) Serial.printf("Subscribed to: %s\n", subscribeTopic);
      else Serial.println("Failed to subscribe!");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("retrying in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// Setup Wifi
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else Serial.println("\nWiFi connection failed!");
}

// MQTT callback function untuk menerima pesan
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  Serial.print("Payload: ");
  Serial.println(message);
  
  // Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // TO DO: Simpan data ke receiveData dengan mutex 
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    newDataReceived = true;

    // Jangan lupa release mutex ddan sertain error handling (isi data yang udah diambil ditampilin ke serial)
  }
}

// Publish data ke MQTT
void publishData() {
  if (!mqttClient.connected()) return;
  
  StaticJsonDocument<256> doc;
  
  // TO DO: isi JSON buat data sendData yang mau dikirim
  // Jangan lupa kalo mau akses data sendData, pake xSemaphoreTake dan kalo udah selese, di xSemaphoreGive
  // sabeb siapa

  // TO DO: Serialize JSON ke string

  if (mqttClient.publish(publishTopic, jsonString.c_str())) {
      // TO DO: Publish ke MQTT dan sertain error handling
    
  } else Serial.println("Publish failed!");
}

// Process data yang diterima
void processReceivedData() {
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    // TO DO: Check if MAC address from website match with our ESP
    // hint: XSemaphoreGive
    // Jafor
    
    // Start timer jika duration > 0
    // TO DO: Timer logic
    // Jangan lupa kalo udh selesai timernya, dia langsung publish data lagi
    // Andrew
    
    newDataReceived = false;
    xSemaphoreGive(dataMutex);
  }
}


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
