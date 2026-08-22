# ESP devices

Multi scripts for different type of devices example.
Include an ESP32 relay card library to be reused among devices with relays.

## ESP32RelayCard library

Reusable PlatformIO library for ESP32-C3 IoT cards.  
Drop it in any project's `lib/` folder and define your card in a single config struct.

---

### Features

| Feature  | Details                                              |
| -------- | ---------------------------------------------------- |
| WiFi     | WiFiManager captive-portal, persisted credentials    |
| REST API | ESPAsyncWebServer, extensible via `card->webServer()` |
| Relays   | N relays, pulse or latching, named routes  |
| MQTT     | PubSubClient, auto-reconnect, config saved in NVS    |
| NeoPixel | Status LED (green=boot, blue=ready, red=error)       |

### Minimal usage

```cpp
#include "ESP32CardLib.h"
#define PIN_RELAY_TOGGLE 1

CardConfig cfg;
ESP32RelayCard* card = nullptr;

void setup() {
  cfg.apName = "ESP32Config",
  cfg.apPassword = "ESP32Config123",
  cfg.mqttClientId = "client-toggle-1",
  cfg.relays = { RelayConfig(PIN_RELAY_TOGGLE, "0", 500) };

  card = new ESP32RelayCard(cfg);
  card->begin();
}

void loop() {
  card->loop();
}
```

### REST endpoints (auto-generated)

```
GET  /                  → help text listing all routes
GET  /reset/wifi        → clear WiFi config and restart
POST /toggle/<name>     → trigger relay by name
POST /config/mqtt       → configure MQTT (JSON body below)
```

---

## ESP-02S Starter Template

A minimal starting point for new ESP-02S (ESP8266) projects. It includes WiFi
setup, mDNS, OTA flashing over WiFi, and remote debugging over telnet — so you
can develop and update a device without touching the serial port.

### Features

| Feature        | Details                                                        |
| -------------- | -------------------------------------------------------------- |
| WiFi           | WiFiManager captive-portal, persisted credentials              |
| mDNS           | Advertised as `http://esp-starter-<chipid>.local`              |
| OTA            | ArduinoOTA — flash over WiFi from PlatformIO or Arduino IDE    |
| Remote debug   | RemoteDebug library — live logs over telnet (port 23)          |
| Web server     | Help page (`/`) and status endpoint (`/status`)                |
| Status LED     | Onboard LED (GPIO 2) blinks once per second                    |

### Build & flash (serial)

```bash
pio run -e starter_esp02s -t upload
```

### Flash over WiFi (OTA)

First flash the device over serial once (see above). After that, the device
advertises itself via ArduinoOTA. To flash over WiFi:

```bash
pio run -e starter_esp02s_ota -t upload --upload-port <device-ip>
```

The device IP is shown in the serial output on boot, or you can query it via
`http://esp-starter-<chipid>.local/status`.

### Remote debugging (telnet)

Connect to the device's telnet server to see live debug output:

```bash
telnet <device-ip> 23
```

Once connected, type `help` to see available commands (set debug level, reset
the device, etc.). Output is also mirrored to the serial monitor.
