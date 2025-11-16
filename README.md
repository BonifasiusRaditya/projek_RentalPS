# ESP32 MQTT IoT Controller

This project is an ESP32-based IoT device controller using MQTT protocol and FreeRTOS.

## Features

- 🌐 **MQTT Communication** - Subscribe and publish to specific topics
- ⚡ **FreeRTOS** - Multi-task operation for better performance
- 🔒 **Security** - MAC address filtering for device validation
- ⏰ **Timer Control** - Automatic device shutdown after specified duration
- 🔄 **Auto-reconnect** - Automatic WiFi and MQTT reconnection

## Hardware Requirements

- ESP32 Development Board
- Any controllable device (LED, relay, etc.) connected to pin 33

## MQTT Topics

### Subscribe Topic: `T3nz/UPN/coffeeShop/getData`
Receives control commands in JSON format:
```json
{
  "mac": "AA:BB:CC:DD:EE:FF",
  "state": true,
  "duration": 5
}
```

### Publish Topic: `T3nz/UPN/coffeeShop/sendData`
Sends device status in JSON format:
```json
{
  "mac": "AA:BB:CC:DD:EE:FF",
  "state": true
}
```

## Configuration

Edit the following values in `src/main.cpp`:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "your.mqtt.broker.com";
```

## Build and Upload

1. Install PlatformIO IDE
2. Open this project
3. Configure your WiFi credentials
4. Build and upload:
   ```bash
   pio run --target upload
   ```

## Monitor Serial Output

```bash
pio device monitor
```

## Dependencies

- PubSubClient (MQTT)
- ArduinoJson (JSON parsing)
- WiFi (ESP32 built-in)

All dependencies are automatically managed by PlatformIO.
