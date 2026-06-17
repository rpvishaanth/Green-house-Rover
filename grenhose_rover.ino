/*
 * Energy-Efficient Rover for Greenhouse Farming
 */

// --- BLYNK V2 CREDENTIALS (MUST BE AT THE VERY TOP) ---
#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Greenhouse Rover"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_TOKEN"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

// --- Network Credentials ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "broker.hivemq.com";
const int udpPort = 1234;

// --- Pin Definitions (From README) ---
#define DHTPIN 4
#define DHTTYPE DHT22
#define BMP_SDA 21
#define BMP_SCL 22
#define LDR_PIN 34
#define IR_LEFT_PIN 35
#define IR_RIGHT_PIN 32

// Motors (L298N)
#define MOTOR_L_IN1 25
#define MOTOR_L_IN2 26
#define MOTOR_R_IN3 27
#define MOTOR_R_IN4 14

// Relays (Active LOW)
#define RELAY_FAN 16
#define RELAY_PUMP 17
#define RELAY_LIGHT 18
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// --- Global Objects ---
WiFiUDP udp;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;

// --- Shared Data & RTOS Primitives ---
struct SensorData {
  float temp;
  float humidity;
  float pressure;
  int light;
  int irLeft;
  int irRight;
  
  bool fanState;
  bool pumpState;
  bool lightState;
  
  bool anomaly;
  bool autoMode;
};
SensorData roverData;

SemaphoreHandle_t xMutexSensor;
SemaphoreHandle_t xSemFwd;
SemaphoreHandle_t xSemRev;
SemaphoreHandle_t xSemLeft;
SemaphoreHandle_t xSemRight;
SemaphoreHandle_t xSemStop;

// Z-Score Buffer
const int WINDOW_SIZE = 10;
float tempBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
bool bufferFilled = false;

// --- Task Declarations ---
void Task1_SensorRead(void *pvParameters);
void Task2_AnomalyDetect(void *pvParameters);
void Task4_Actuator(void *pvParameters);
void Task5_UDPListener(void *pvParameters);
void Task5_MotorControl(void *pvParameters);
void Task6_MQTTPublish(void *pvParameters);
void Task7_BlynkSync(void *pvParameters);
void Task8_LineFollow(void *pvParameters);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin(BMP_SDA, BMP_SCL);
  
  // Initialize Pins
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT); pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT); pinMode(MOTOR_R_IN4, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT); pinMode(RELAY_PUMP, OUTPUT); pinMode(RELAY_LIGHT, OUTPUT);
  
  // Safe Startup State
  digitalWrite(RELAY_FAN, RELAY_OFF);
  digitalWrite(RELAY_PUMP, RELAY_OFF);
  digitalWrite(RELAY_LIGHT, RELAY_OFF);
  stopMotors();

  // Initialize Sensors
  dht.begin();
  if (!bmp.begin(0x76)) { Serial.println("BMP280 init failed!"); }

  // Network Init
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  udp.begin(udpPort);
  mqttClient.setServer(mqtt_server, 1883);
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Create RTOS Primitives
  xMutexSensor = xSemaphoreCreateMutex();
  xSemFwd = xSemaphoreCreateBinary();
  xSemRev = xSemaphoreCreateBinary();
  xSemLeft = xSemaphoreCreateBinary();
  xSemRight = xSemaphoreCreateBinary();
  xSemStop = xSemaphoreCreateBinary();

  // --- Task Scheduling (Per README Spec) ---
  
  // Core 1: Sensing & Logic
  xTaskCreatePinnedToCore(Task1_SensorRead, "Task1_Sensor", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task2_AnomalyDetect, "Task2_Anomaly", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task4_Actuator, "Task4_Actuator", 2048, NULL, 2, NULL, 1); // Round-Robin falls under same priority bracket
  xTaskCreatePinnedToCore(Task6_MQTTPublish, "Task6_MQTT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task7_BlynkSync, "Task7_Blynk", 4096, NULL, 1, NULL, 1);

  // Core 0: Comms & Motor Control
  xTaskCreatePinnedToCore(Task5_UDPListener, "Task5_UDP", 2048, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(Task5_MotorControl, "Task5_Motors", 2048, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(Task8_LineFollow, "Task8_Auto", 2048, NULL, 4, NULL, 0);
}

void loop() {
  Blynk.run(); // Maintain Blynk connection
  mqttClient.loop(); // Maintain MQTT connection
  vTaskDelay(pdMS_TO_TICKS(10)); // Yield to RTOS
}

// --- Task 1: Sensor Acquisition (Core 1) ---
void Task1_SensorRead(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      roverData.temp = dht.readTemperature();
      roverData.humidity = dht.readHumidity();
      roverData.pressure = bmp.readPressure() / 100.0F; // hPa
      roverData.light = analogRead(LDR_PIN);
      roverData.irLeft = digitalRead(IR_LEFT_PIN);
      roverData.irRight = digitalRead(IR_RIGHT_PIN);
      xSemaphoreGive(xMutexSensor);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// --- Task 2: Rolling Z-Score Anomaly Detection (Core 1) ---
void Task2_AnomalyDetect(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      float currentTemp = roverData.temp;
      float currentHum = roverData.humidity;
      bool isAnomaly = false;

      // 1. Hard-Range Sanity Check
      if (isnan(currentTemp) || isnan(currentHum) || 
          currentTemp < -10.0 || currentTemp > 80.0 || 
          currentHum < 0.0 || currentHum > 100.0) {
        isAnomaly = true;
      } 
      // 2. Statistical Z-Score (3σ)
      else {
        // Update circular buffer
        tempBuffer[bufferIndex] = currentTemp;
        bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
        if (bufferIndex == 0) bufferFilled = true;

        if (bufferFilled) {
          float sum = 0, mean = 0, variance = 0, stdDev = 0;
          
          // Calculate Mean
          for (int i = 0; i < WINDOW_SIZE; i++) sum += tempBuffer[i];
          mean = sum / WINDOW_SIZE;

          // Calculate Standard Deviation
          for (int i = 0; i < WINDOW_SIZE; i++) {
            variance += pow(tempBuffer[i] - mean, 2);
          }
          stdDev = sqrt(variance / WINDOW_SIZE);

          // Check if current reading is > 3σ from mean
          if (stdDev > 0 && abs(currentTemp - mean) > (3 * stdDev)) {
            isAnomaly = true;
          }
        }
      }

      roverData.anomaly = isAnomaly;
      xSemaphoreGive(xMutexSensor);

      // Trigger MQTT alert immediately if anomaly detected
      if (isAnomaly) {
        mqttClient.publish("greenhouse/rover/status", "{\"alert\":\"SENSOR_ANOMALY\"}");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// --- Task 4: Actuator Control (Active LOW) (Core 1) ---
void Task4_Actuator(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      if (roverData.anomaly) {
        // Safe-off state on anomaly
        digitalWrite(RELAY_FAN, RELAY_OFF);
        digitalWrite(RELAY_PUMP, RELAY_OFF);
        digitalWrite(RELAY_LIGHT, RELAY_OFF);
        roverData.fanState = roverData.pumpState = roverData.lightState = false;
      } else {
        // Threshold Logic
        roverData.fanState = (roverData.temp > 35.0);
        digitalWrite(RELAY_FAN, roverData.fanState ? RELAY_ON : RELAY_OFF);

        roverData.lightState = (roverData.light < 1000);
        digitalWrite(RELAY_LIGHT, roverData.lightState ? RELAY_ON : RELAY_OFF);
        
        // Example Pump threshold based on Humidity if soil sensor isn't present
        roverData.pumpState = (roverData.humidity < 40.0);
        digitalWrite(RELAY_PUMP, roverData.pumpState ? RELAY_ON : RELAY_OFF);
      }
      xSemaphoreGive(xMutexSensor);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --- Task 5: UDP Listener (Core 0) ---
void Task5_UDPListener(void *pvParameters) {
  char packetBuffer[10];
  for (;;) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 10);
      if (len > 0) packetBuffer[len] = 0;
      char cmd = packetBuffer[0];
      
      if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
        if (cmd == 'A') roverData.autoMode = true;
        else roverData.autoMode = false;
        xSemaphoreGive(xMutexSensor);
      }

      // Signal Semaphores
      switch(cmd) {
        case 'F': xSemaphoreGive(xSemFwd); break;
        case 'B': xSemaphoreGive(xSemRev); break;
        case 'L': xSemaphoreGive(xSemLeft); break;
        case 'R': xSemaphoreGive(xSemRight); break;
        case 'S': xSemaphoreGive(xSemStop); break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Task 5: Motor Execution (Core 0) ---
void Task5_MotorControl(void *pvParameters) {
  for (;;) {
    bool isAuto = false;
    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      isAuto = roverData.autoMode;
      xSemaphoreGive(xMutexSensor);
    }

    if (!isAuto) {
      if (xSemaphoreTake(xSemFwd, 0)) moveForward();
      else if (xSemaphoreTake(xSemRev, 0)) moveBackward();
      else if (xSemaphoreTake(xSemLeft, 0)) turnLeft();
      else if (xSemaphoreTake(xSemRight, 0)) turnRight();
      else if (xSemaphoreTake(xSemStop, 0)) stopMotors();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- Task 8: Line Follow Logic (Core 0) ---
void Task8_LineFollow(void *pvParameters) {
  for (;;) {
    bool isAuto = false;
    int lIR = 0, rIR = 0;

    if (xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      isAuto = roverData.autoMode;
      lIR = roverData.irLeft;
      rIR = roverData.irRight;
      xSemaphoreGive(xMutexSensor);
    }

    if (isAuto) {
      if (lIR == LOW && rIR == LOW) moveForward();
      else if (lIR == HIGH && rIR == LOW) turnLeft();
      else if (lIR == LOW && rIR == HIGH) turnRight();
      else stopMotors();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Task 6: MQTT Telemetry Publish (Core 1) ---
void Task6_MQTTPublish(void *pvParameters) {
  for (;;) {
    if (mqttClient.connected() && xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      char payload[256];
      snprintf(payload, sizeof(payload), 
        "{\"temp\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"light\":%d,\"fan\":%d,\"pump\":%d,\"light_relay\":%d,\"anomaly\":%d}",
        roverData.temp, roverData.humidity, roverData.pressure, roverData.light, 
        roverData.fanState, roverData.pumpState, roverData.lightState, roverData.anomaly);
        
      mqttClient.publish("greenhouse/rover/sensors", payload);
      xSemaphoreGive(xMutexSensor);
    } else if (!mqttClient.connected()) {
      mqttClient.connect("ESP32RoverClient");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// --- Task 7: Blynk Sync (Core 1) ---
void Task7_BlynkSync(void *pvParameters) {
  for (;;) {
    if (Blynk.connected() && xSemaphoreTake(xMutexSensor, portMAX_DELAY)) {
      Blynk.virtualWrite(V0, roverData.temp);
      Blynk.virtualWrite(V1, roverData.humidity);
      Blynk.virtualWrite(V2, roverData.pressure);
      Blynk.virtualWrite(V3, roverData.light);
      Blynk.virtualWrite(V4, roverData.fanState);
      Blynk.virtualWrite(V5, roverData.pumpState);
      Blynk.virtualWrite(V6, roverData.lightState);
      Blynk.virtualWrite(V7, roverData.anomaly);
      xSemaphoreGive(xMutexSensor);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// --- Hardware Motor Helper Functions ---
void moveForward()  { digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_R_IN3, HIGH); digitalWrite(MOTOR_L_IN2, LOW); digitalWrite(MOTOR_R_IN4, LOW); }
void moveBackward() { digitalWrite(MOTOR_L_IN2, HIGH); digitalWrite(MOTOR_R_IN4, HIGH); digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_R_IN3, LOW); }
void turnLeft()     { digitalWrite(MOTOR_L_IN2, HIGH); digitalWrite(MOTOR_R_IN3, HIGH); digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_R_IN4, LOW); }
void turnRight()    { digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_R_IN4, HIGH); digitalWrite(MOTOR_L_IN2, LOW); digitalWrite(MOTOR_R_IN3, LOW); }
void stopMotors()   { digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_R_IN3, LOW); digitalWrite(MOTOR_L_IN2, LOW); digitalWrite(MOTOR_R_IN4, LOW); }
