#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Sensor & Actuator Pins ---
#define DHTPIN 15
#define DHTTYPE DHT22
#define LDR_PIN 34
#define SOIL_PIN 35
#define TRIG_PIN 14
#define ECHO_PIN 12
#define RGB_PIN 5
#define SERVO_PIN 18
#define WIFI_LED 23
#define MQTT_LED 17
#define BUTTON_PIN 16
#define FAN_RELAY_PIN 27
#define PUMP_RELAY_PIN 26
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- WiFi & MQTT Credentials ---
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "YOUR_SERVER";
const int mqtt_port = 8883;
const char* mqtt_user = "YOUR_CREDENTIALS";
const char* mqtt_pass = "YOUR_PASSWORD";

// --- MQTT Topics ---
const char* temp_topic = "greenhouse/sensors/temperature";
const char* humidity_topic = "greenhouse/sensors/humidity";
const char* soilmoisture_topic = "greenhouse/sensors/soilMoisture";
const char* light_topic = "greenhouse/sensors/light";
const char* waterlevel_topic = "greenhouse/sensors/waterLevel";
const char* status_topic = "greenhouse/status";
const char* fan_topic = "greenhouse/control/fan";
const char* pump_topic = "greenhouse/control/pump";
const char* light_control_topic = "greenhouse/control/light";
const char* window_topic = "greenhouse/control/window";

// --- Globals ---
WiFiClientSecure espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
Servo windowServo;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool manualMode = true;

float temperature = 0;
float humidity = 0;
int soil = 0;
int ldr = 0;
float distance = 0;
String alarmstatus;

TaskHandle_t OledTaskHandle = NULL;

volatile bool buttonPressed = false;
volatile unsigned long buttonPressTime = 0;
volatile bool buttonReleased = false;

// --- FreeRTOS Tasks ---
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
TaskHandle_t ButtonTaskHandle = NULL;

//---Interrupt Handler---
void IRAM_ATTR handleButtonInterrupt() {
  bool state = digitalRead(BUTTON_PIN);
  if (state == LOW) {
    buttonPressTime = millis();
    buttonPressed = true;
  } else {
    buttonReleased = true;
  }
}

// --- Setup WiFi ---
void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(WIFI_LED, LOW);
    delay(500);
    Serial.print(".");
  }
  digitalWrite(WIFI_LED, HIGH);
  Serial.println("\nWiFi connected");
}

// --- MQTT Reconnect ---
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp32client", mqtt_user, mqtt_pass)) {
      Serial.println("connected to MQTT broker");
      digitalWrite(MQTT_LED, HIGH);
      client.subscribe(fan_topic);
      client.subscribe(pump_topic);
      client.subscribe(light_control_topic);
      client.subscribe(window_topic);
    } else {
      digitalWrite(MQTT_LED, LOW);
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// --- MQTT Callback ---
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  Serial.printf("Message arrived on topic %s: %s\n", topic, message.c_str());
if(manualMode){
  if (strcmp(topic, light_control_topic) == 0) {
    int value = message.toInt() * 2.56;
    ledcWrite(6, value);
  } else if (strcmp(topic, window_topic) == 0) {
    int angle = message.toInt();
    windowServo.write(angle);
  } else if (strcmp(topic, fan_topic) == 0) {
    digitalWrite(FAN_RELAY_PIN, message == "true" ? HIGH : LOW);
  } else if (strcmp(topic, pump_topic) == 0) {
    digitalWrite(PUMP_RELAY_PIN, message == "true" ? HIGH : LOW);
  }
}
}

// --- Task: Sensor Publishing ---
void SensorTask(void* parameter) {
  for (;;) {
    if (!client.connected()) reconnect();

    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    soil = analogRead(SOIL_PIN) / 40.96;
    ldr = 100 - analogRead(LDR_PIN) / 40.96;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    distance =400- duration * 0.034 / 2;

    alarmstatus = (distance < 5) ? "Low Water" : "Running";

    client.publish(temp_topic, String(temperature, 2).c_str());
    client.publish(humidity_topic, String(humidity, 2).c_str());
    client.publish(soilmoisture_topic, String(soil).c_str());
    client.publish(light_topic, String(ldr).c_str());
    client.publish(waterlevel_topic, String(distance, 1).c_str());
    client.publish(status_topic, alarmstatus.c_str());

    Serial.println("Sensor data published");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// --- Task: Actuator Listening ---
void ControlTask(void* parameter) {
  for (;;) {
    if (manualMode){
      client.loop();
    } else {
      digitalWrite(FAN_RELAY_PIN, (temperature > 30 || humidity > 70) ? HIGH : LOW);
      if (soil < 30 && distance > 10) {
        digitalWrite(PUMP_RELAY_PIN, HIGH);
        delay(10000);
        digitalWrite(PUMP_RELAY_PIN, LOW);
      }

      ledcWrite(6, ldr < 50 ? 255 : (ldr > 70 ? 0 : ledcRead(6)));

      windowServo.write((temperature > 30 && humidity > 50) ? 90 : 0);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// --- Task: Button Monitor ---
void ButtonTask(void* parameter) {
  for (;;) {
    if (buttonReleased) {
      unsigned long pressDuration = millis() - buttonPressTime;
      buttonReleased = false;

      if (pressDuration >= 5000) {
        Serial.println("Button held for 5s. Restarting ESP...");
        ESP.restart();
      } else if (buttonPressTime > 100) {
        manualMode = !manualMode;
        Serial.print("Mode changed to: ");
        Serial.println(manualMode ? "Manual" : "Automatic");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void OledTask(void* parameter) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (;;) {
    display.clearDisplay();

    display.setCursor(0, 0);
    display.printf("Mode: %s\n", manualMode ? "Manual" : "Automatic");
    display.printf("Temp: %.1f C\n", temperature);
    display.printf("Humidity: %.1f %%\n", humidity);
    display.printf("Light: %d %%\n", ldr);
    display.printf("Soil: %d %%\n", soil);
    display.printf("Water: %.1f cm\n", distance);
    if (distance < 5) display.printf("LOW WATER\n");

    display.display();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(WIFI_LED, OUTPUT);
  pinMode(MQTT_LED, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(PUMP_RELAY_PIN, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  ledcSetup(6, 5000, 8);
  ledcAttachPin(RGB_PIN, 6);
  ledcWrite(6, 0);

  dht.begin();
  windowServo.setPeriodHertz(50);
  if (!windowServo.attach(SERVO_PIN, 500, 2500)) windowServo.attach(SERVO_PIN);
  windowServo.write(0);
  delay(1000);
  windowServo.write(90);
  delay(1000);
  windowServo.write(180);
  delay(1000);

  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, CHANGE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Smart Greenhouse");
  display.display();
  delay(1000);

  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 8192, NULL, 1, &SensorTaskHandle, 0);
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL, 1, &ControlTaskHandle, 1);
  xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", 2048, NULL, 1, &ButtonTaskHandle, 1);
  xTaskCreatePinnedToCore(OledTask, "OledTask", 4096, NULL, 1, &OledTaskHandle, 1);
}

void loop() {
  // Everything is handled in FreeRTOS tasks
}