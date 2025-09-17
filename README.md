# Smart Greenhouse – ESP32 + MQTT + Node-RED (Wokwi / PlatformIO)

FreeRTOS-based ESP32 greenhouse: publishes sensor data, accepts manual actuator commands via MQTT/Node-RED, and supports an automatic control mode. Simulatable on Wokwi; built with PlatformIO.

## Features
- DHT22, LDR, soil moisture, ultrasonic water level, OLED dashboard
- Manual + automatic modes (button toggled, long-press resets)
- MQTT over TLS (port 8883), Node-RED dashboard
- Wokwi simulation and PlatformIO build
- Clear topic contract (see below)

## Hardware & Pins
- DHT22 on GPIO 15
- LDR (ADC) GPIO 34, Soil (ADC) GPIO 35
- Ultrasonic TRIG 14 / ECHO 12
- RGB PWM: GPIO 5 (channel 6)
- Servo: GPIO 18
- Fan relay: 27, Pump relay: 26
- WiFi LED: 23, MQTT LED: 17, Button: 16
- OLED I²C: SDA 21 / SCL 22

## MQTT Topics
Sensors (publish):
- greenhouse/sensors/temperature (°C)
- greenhouse/sensors/humidity (%)
- greenhouse/sensors/soilMoisture (%)
- greenhouse/sensors/light (%)
- greenhouse/sensors/waterLevel (cm; higher = fuller)
- greenhouse/status (Running | Low Water)
Actuators (subscribe when manual mode):
- greenhouse/control/fan (true/false)
- greenhouse/control/pump (true/false)
- greenhouse/control/light (0–100 → PWM)
- greenhouse/control/window (0–180 → servo angle)
