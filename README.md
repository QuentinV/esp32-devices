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

---

## ESP-02S Shutter Controller

A shutter/roller-blind controller for the ESP-02S (ESP8266). In addition to its
WiFi + mDNS + MQTT + REST API it supports OTA flashing over WiFi and remote
debugging over telnet (same setup as the Starter template).

### Features

| Feature        | Details                                                          |
| -------------- | ---------------------------------------------------------------- |
| WiFi           | WiFiManager captive-portal, persisted credentials                |
| mDNS           | Advertised as `http://esp-shutter-<chipid>.local`                |
| OTA            | ArduinoOTA — flash over WiFi from PlatformIO or Arduino IDE      |
| Remote debug   | RemoteDebug library — live logs over telnet (port 23)            |
| MQTT           | PubSubClient — command/state topics, config saved in EEPROM      |
| REST API       | `/state`, `/move`, `/pause`, `/home`, `/calibrate`, `/config/mqtt` |

### Build & flash (serial)

```bash
pio run -e shutter_esp02s -t upload
```

### Flash over WiFi (OTA)

First flash the device over serial once (see above). After that, the device
advertises itself via ArduinoOTA. To flash over WiFi:

```bash
pio run -e shutter_esp02s_ota -t upload --upload-port <device-ip>
```

The device IP is printed on boot (serial), or query it via
`http://esp-shutter-<chipid>.local/state`.

### Remote debugging (telnet)

Connect to the device's telnet server to see live debug output:

```bash
telnet <device-ip> 23
```

Once connected, type `help` to see available commands (set debug level, reset
the device, etc.). Output is mirrored to the serial monitor as well.

---

## ESP-02S Light Switch

A single light / relay controller for the ESP-02S (ESP8266). In addition to its
WiFi + mDNS + MQTT + REST API it supports OTA flashing over WiFi and remote
debugging over telnet (same setup as the Starter template).

### Features

| Feature        | Details                                                          |
| -------------- | ---------------------------------------------------------------- |
| WiFi           | WiFiManager captive-portal, persisted credentials                |
| mDNS           | Advertised as `http://esp-light-<chipid>.local`                  |
| OTA            | ArduinoOTA — flash over WiFi from PlatformIO or Arduino IDE      |
| Remote debug   | RemoteDebug library — live logs over telnet (port 23)            |
| MQTT           | PubSubClient — publish state, config saved in EEPROM             |
| REST API       | `/state`, `/toggle`, `/config/mqtt`, `/reset/wifi`              |
| Physical input | Debounced switch (GPIO 4, active LOW) toggles the relay locally  |

### Build & flash (serial)

```bash
pio run -e light_switch -t upload
```

### Flash over WiFi (OTA)

First flash the device over serial once (see above). After that, the device
advertises itself via ArduinoOTA. To flash over WiFi:

```bash
pio run -e light_switch_esp02s_ota -t upload --upload-port <device-ip>
```

The device IP is printed on boot (serial), or query it via
`http://esp-light-<chipid>.local/`.

### Remote debugging (telnet)

Connect to the device's telnet server to see live debug output:

```bash
telnet <device-ip> 23
```

Once connected, type `help` to see available commands (set debug level, reset
the device, etc.). Output is mirrored to the serial monitor as well.
---

## ESP32-C3 Micronova Stove Monitor

Interface with the serial bus of a pellet stove with a Micronova controller,
from an ESP32-C3. The Micronova board is a **slave**: it never sends anything
on its own, so this firmware now works as an **interrogator** (master) that
sends read requests and decodes the stove's replies.

### Hardware

- ESP32-C3 — UART0: RX = GPIO20, TX = GPIO21
- **Non-inverting 2-channel optocoupler wiring** (keeps TX and RX invert flags
  at 0):
  - **RX channel (emitter follower):** stove data wire → resistor (~1k) → LED →
    stove GND; transistor **collector → +3.3 V**, **emitter → GPIO20**. Requires
    `INPUT_PULLDOWN` on GPIO20 (`STOVE_RX_PULLMODE`).
  - **TX channel (active-low LED):** GPIO21 → resistor (~470Ω) → LED **cathode**,
    LED **anode → +3.3 V** (the LED lights when GPIO21 goes low = start bit, so
    it is off at idle); transistor collector → stove data wire, emitter → stove
    GND.
- No `enable_rx` optocoupler: our own TX echo on the shared wire is drained
  in software (`ECHO_DRAIN_MS`) after each emission.
- **During tests, unplug the stove WiFi/console module** (two masters on the
  single-wire bus would collide).

### Protocol

Micronova mainboards use a single-wire serial bus at **1200 baud**, 8 data
bits, no parity, **2 stop bits** (8N2).

Read request (2 bytes): `[0x00] [addr]` = RAM, `[0x20] [addr]` = EEPROM.
Reply (2 bytes): `[addr+value] [value]`. The board answers in ~120 ms.

### Build & flash

```bash
pio run -e micronova -t upload
```

Then open the serial monitor at 115200 baud.

### Firmware behavior

- Every 5 seconds the ESP sends read requests for the addresses in
  `stoveReads[]` (state, ambient temp, fumes temp, power, fan rpm, thermostat)
  and prints a debug block with each decoded value plus a checksum validation.
- Values whose checksum does not match are also printed raw (hex) so that the
  address mapping can be tuned for a specific stove model (e.g. **MUSA C/AIR**).
- The diagnostic block also reports signal transitions: if `transitions > 0`
  but no reply, our TX is driving the wire but the board is not answering
  (check TX wiring, stove power, `STOVE_INVERT_TX`/`STOVE_INVERT_RX`).

### Scan mode (find the right UART config + map the stove)

If the stove does not answer on the default setting, the built-in scanner
finds the right combination automatically. When `STOVE_SCAN_MODE 1`, at boot
the ESP:

1. **Scans UART combinations** — 1200/2400/4800/9600 bauds × 8N1/8N2 ×
   RX normal/inverted — each evaluated by a passive listen + 4 probe reads.
   The best combo (any decoded bytes, ideally with a valid checksum) is kept.
2. **Sweeps the addresses** of RAM (`0x00`) and EEPROM (`0x20`) from `0x00` to
   `0x7F`, printing every response with its raw bytes and checksum status.

The watchdog is fed during the scan (`esp_task_wdt_reset()`) so it can run
for minutes safely.

### Serial menu (interactive)

After boot (or just type `?`), the USB monitor accepts single-key commands:

```
?  help                 s  full scan (configs + RAM/EEPROM addresses)
c  scan configs only    a  sweep addresses (current config)
e  echo test (loopback) p  polling mode
1-6  set baud/format    r/t  toggle RX/TX inversion
b  show current config  v  poll once now
```

Example: type `5` to try `9600 bauds 8N2`, then `a` to sweep the addresses
under that setting, then `b` to confirm what is active.

### Configuration switches

| Macro | Default | Meaning |
|---|---|---|
| `STOVE_USE_TX` | 1 | 1 = interrogator mode (TX active). Set to 0 to revert to passive sniff |
| `STOVE_INVERT_TX` | 0 | Invert the TX line (1 = open-collector LED wiring) |
| `STOVE_INVERT_RX` | 0 | Invert the RX line (1 = open-collector transistor wiring) |
| `STOVE_RX_PULLMODE` | `INPUT_PULLDOWN` | RX pin mode: `INPUT_PULLDOWN` for emitter follower, `INPUT_PULLUP` for open collector |
| `DIAGNOSTIC_MODE` | 1 | Enable the 5 s diagnostic block + waveform capture |
| `ECHO_DRAIN_MS` | 40 | Software echo-drain window after each TX request |
| `REPLY_TIMEOUT_MS` | 300 | Max time to collect the stove reply |
| `ECHO_TEST` | 0 | 1 = send a known burst every cycle and count received bytes (loopback opto validation without the stove) |
| `STOVE_SCAN_MODE` | 1 | Run the UART config scan + address sweep at boot |
| `SCAN_LISTEN_MS` | 400 | Passive listen duration per tested combination |
| `SCAN_PROBE_MS` | 220 | Receive window after each probe request |
| `SCAN_ADDR_MAX` | `0x7F` | Last address scanned (RAM & EEPROM) |

The serial config line is now **computed from the actual macros/pin mode**
(no hardcoded "pull-up"), e.g. `RX=normal,pull-down GPIO20 | TX=normal ACTIF`.

Known read addresses (eni23 / ESPHome mappings, tune for your stove): RAM
`0x21` state, `0x01` ambient temp (`/2` = °C), `0x3E` fumes temp, `0x34`
power, `0x37` fan rpm (`×10`); EEPROM `0x7D` thermostat setpoint.
