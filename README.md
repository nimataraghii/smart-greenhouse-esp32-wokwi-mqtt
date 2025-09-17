# smart-greenhouse-esp32-wokwi-mqtt
ESP32 greenhouse monitoring and controlling system with FreeRTOS tasks, MQTT + HiveMQ, Node-RED dashboard, Wokwi simulation, built with PlatformIO

MQTT Topics:
Sensors (publish):

greenhouse/sensors/temperature (°C)

greenhouse/sensors/humidity (%)

greenhouse/sensors/soilMoisture (%)

greenhouse/sensors/light (%)

greenhouse/sensors/waterLevel (cm; higher = fuller)

greenhouse/status (Running | Low Water)

Actuators (subscribe when manual mode):

greenhouse/control/fan (true/false)

greenhouse/control/pump (true/false)

greenhouse/control/light (0–100 → PWM)

greenhouse/control/window (0–180 → servo angle)

