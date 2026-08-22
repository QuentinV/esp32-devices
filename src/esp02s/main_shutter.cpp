#include <Arduino.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <PubSubClient.h>

// ── Pin configuration ────────────────────────────
#define PIN_TOUCH_UP      14   // touch sensor (HIGH when touched)
#define PIN_TOUCH_PAUSE   12   // touch sensor (HIGH when touched)
#define PIN_TOUCH_DOWN    5    // touch sensor (HIGH when touched)
#define PIN_RELAY_ENABLE  3    // enable relay (must be ON first)
#define PIN_RELAY_DIR     4    // direction relay (ON=UP, OFF=DOWN)
#define PIN_ADC           A0   // ACS712 current sensor (0-1V via divider)

// ── Timing / thresholds ──────────────────────────
#define DEBOUNCE_MS        50
#define RELAY_SETTLE_MS    50    // delay between direction relay and enable relay
#define ADC_SAMPLES        16    // moving average window
#define STALL_THRESHOLD    30    // ADC delta above offset = motor drawing current
#define STALL_MULTIPLIER   1.6f  // stall = current 60% above steady running baseline
#define STALL_HOLD_MS      400   // how long the spike must persist before auto-stop
#define BASELINE_SAMPLES   8     // samples (~0.5s) to establish running baseline
#define MQTT_EEPROM_ADDR   0
#define ADC_OFFSET_ADDR    300   // float (4 bytes)

// ACS712 estimate (approx. — the app will recalibrate later)
#define ADC_DIVIDER_RATIO  0.4125f  // 3.3k / (4.7k + 3.3k)
#define ACS_SENSITIVITY    0.185f   // V/A (ACS712-5A)
#define ACS_ZERO_V         2.5f     // ACS712 output at 0A

// ── Globals ──────────────────────────────────────
ESP8266WebServer server(80);
WiFiClient       espClient;
PubSubClient     mqtt(espClient);

// Shutter state
enum ShutterState { IDLE, MOVING_UP, MOVING_DOWN };
ShutterState shutterState = IDLE;
int  position = 0;          // 0 = closed, 100 = open
bool positionValid = false; // false until calibrated/homed

// MQTT config (persisted in EEPROM)
String mqttServer;
int    mqttPort   = 1883;
String mqttTopic;           // command topic (subscribe)
String mqttStateTopic;      // state topic (publish)
String mqttClientId;

// ADC / current
float adcOffset = 0.0f;     // zero-current ADC offset
float adcAvg    = 0.0f;     // filtered ADC value
unsigned long stallStart = 0;
float moveBaseline = 0.0f;  // steady running current baseline (stall detection)
int   baselineSamples = 0;  // samples used to establish the baseline

// Debounce
bool    lastUpRaw = LOW, lastPauseRaw = LOW, lastDownRaw = LOW;
bool    upPressed = false, pausePressed = false, downPressed = false;
unsigned long lastDebounceTime = 0;

// MQTT reconnect throttle
unsigned long lastMqttAttempt = 0;

// Forward declaration (defined later in the file)
void mqttPublishState();

// ── Helpers ──────────────────────────────────────
void setCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin",  "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendJSON(int code, const String& payload) {
    setCORSHeaders();
    server.send(code, "application/json", payload);
}

String directionName() {
    switch (shutterState) {
        case MOVING_UP:   return "up";
        case MOVING_DOWN: return "down";
        default:          return "idle";
    }
}

// ── Relay control (interlock) ────────────────────
// GPIO3 = enable relay, GPIO4 = direction relay (ON=UP, OFF=DOWN)
// UP   → enable ON, direction ON
// DOWN → enable ON, direction OFF
// STOP → both OFF
//
// Safe sequencing:
//   1. Position the direction relay while the motor circuit is unpowered
//      (enable relay still OFF — hardware interlock guarantees this)
//   2. Wait for the contact to settle
//   3. Energize the enable relay to apply motor power
void setRelays(bool enable, bool direction) {
    if (!enable) {
        // Soft stop: direction off first, then enable
        digitalWrite(PIN_RELAY_DIR, LOW);
        digitalWrite(PIN_RELAY_ENABLE, LOW);
        Serial.println("[RELAY] STOP (both off)");
        return;
    }
    digitalWrite(PIN_RELAY_DIR, direction ? HIGH : LOW);
    delay(RELAY_SETTLE_MS);
    digitalWrite(PIN_RELAY_ENABLE, HIGH);
    Serial.printf("[RELAY] %s (enable=ON, dir=%s)\n",
        direction ? "UP" : "DOWN", direction ? "ON" : "OFF");
}

void stopShutter() {
    setRelays(false, false);
    shutterState = IDLE;
    stallStart = 0;
    Serial.println("[SHUTTER] Stopped");
    mqttPublishState();
}

// Change direction safely: always stop first, let the relays settle, then
// re-energize in the new direction. Switching the direction relay while the
// motor power path is live would defeat the hardware interlock.
void moveTo(bool up) {
    ShutterState target = up ? MOVING_UP : MOVING_DOWN;
    if (shutterState == target) {
        return; // already moving that way
    }
    if (shutterState != IDLE) {
        setRelays(false, false);
        delay(RELAY_SETTLE_MS);
    }
    setRelays(true, up);
    shutterState = target;
    stallStart = 0;
    moveBaseline = 0.0f;
    baselineSamples = 0;
    Serial.printf("[SHUTTER] Moving %s\n", up ? "UP" : "DOWN");
    mqttPublishState();
}

void moveUp() { moveTo(true); }
void moveDown() { moveTo(false); }

// ── ADC / current sensing ────────────────────────
float readAdcFiltered() {
    long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PIN_ADC);
        delay(1);
    }
    return (float)sum / ADC_SAMPLES;
}

// Returns estimated current in mA (approximate — app recalibrates later)
float adcToCurrent(float adc) {
    // ESP8266 ADC: 0-1023 maps to 0-1.0V
    float adcVolts = adc / 1023.0f;
    // Restore the ACS712 output voltage before the divider
    float acsVolts = adcVolts / ADC_DIVIDER_RATIO;
    float currentA = (acsVolts - ACS_ZERO_V) / ACS_SENSITIVITY;
    return currentA * 1000.0f; // mA
}

// ── EEPROM persistence ───────────────────────────
void loadMqttConfig() {
    String raw;
    for (int i = MQTT_EEPROM_ADDR; i < 512; i++) {
        char c = EEPROM.read(i);
        if (c == '\0') break;
        raw += c;
    }

    if (raw.length() > 0) {
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, raw);
        if (!err) {
            mqttServer      = doc["server"] | "";
            mqttPort        = doc["port"]   | 1883;
            mqttTopic       = doc["topic"]  | "";
            mqttStateTopic  = doc["state_topic"] | "";
            mqttClientId    = doc["client"] | "";
        }
    }

    if (mqttClientId.length() == 0) {
        mqttClientId = "esp-shutter-" + String((uint32_t)ESP.getChipId(), HEX);
    }

    // Load ADC zero-current offset (float). Fresh EEPROM reads as 0xFF (NaN),
    // so validate before use.
    float loadedOffset;
    EEPROM.get(ADC_OFFSET_ADDR, loadedOffset);
    if (isnan(loadedOffset) || loadedOffset < 0.0f || loadedOffset > 1023.0f) {
        loadedOffset = 0.0f;
    }
    adcOffset = loadedOffset;

    // Position lives in RAM only: unknown until the app re-homes the shutter
    // after a reboot (POST /home). This avoids EEPROM wear entirely.
    position = 0;
    positionValid = false;

    if (mqttServer.length() > 0) {
        mqtt.setServer(mqttServer.c_str(), mqttPort);
        Serial.printf("[MQTT] Config loaded: %s:%d cmd=%s state=%s\n",
            mqttServer.c_str(), mqttPort, mqttTopic.c_str(), mqttStateTopic.c_str());
    }
    Serial.printf("[ADC] Offset=%.1f Position=%d\n", adcOffset, position);
}

void saveMqttConfig(const String& srv, int port, const String& topic, const String& stateTopic) {
    StaticJsonDocument<256> doc;
    doc["server"]      = srv;
    doc["port"]        = port;
    doc["topic"]       = topic;
    doc["state_topic"] = stateTopic;
    doc["client"]      = mqttClientId;

    String json;
    serializeJson(doc, json);

    for (size_t i = 0; i < json.length(); i++) {
        EEPROM.write(MQTT_EEPROM_ADDR + i, json[i]);
    }
    EEPROM.write(MQTT_EEPROM_ADDR + json.length(), '\0');
    EEPROM.commit();
}

void saveAdcOffset(float offset) {
    EEPROM.put(ADC_OFFSET_ADDR, offset);
    EEPROM.commit();
}

// ── MQTT ─────────────────────────────────────────
void mqttPublishState() {
    if (mqttServer.length() == 0) return;
    if (!mqtt.connected()) return;

    StaticJsonDocument<256> doc;
    doc["direction"] = directionName();
    doc["current"]   = adcToCurrent(adcAvg);
    doc["adc"]       = (int)adcAvg;
    doc["position"]  = position;
    doc["valid"]     = positionValid;

    String payload;
    serializeJson(doc, payload);

    if (mqttStateTopic.length() > 0) {
        mqtt.publish(mqttStateTopic.c_str(), payload.c_str(), true);
        Serial.printf("[MQTT] Published to %s: %s\n", mqttStateTopic.c_str(), payload.c_str());
    }
}

void mqttReconnect() {
    if (mqttServer.length() == 0) return;

    Serial.printf("[MQTT] Connecting to %s:%d as %s ...\n",
        mqttServer.c_str(), mqttPort, mqttClientId.c_str());

    if (mqtt.connect(mqttClientId.c_str())) {
        Serial.println("[MQTT] Connected");
        if (mqttTopic.length() > 0) {
            mqtt.subscribe(mqttTopic.c_str());
            Serial.printf("[MQTT] Subscribed to %s\n", mqttTopic.c_str());
        }
        mqttPublishState();
    } else {
        Serial.printf("[MQTT] Failed, rc=%d\n", mqtt.state());
    }
}

void mqttLoop() {
    if (mqttServer.length() == 0) return;

    if (mqtt.connected()) {
        mqtt.loop();
    } else {
        unsigned long now = millis();
        if (now - lastMqttAttempt > 10000) {
            lastMqttAttempt = now;
            mqttReconnect();
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    msg.toUpperCase();
    Serial.printf("[MQTT] Command: %s\n", msg.c_str());

    if (msg == "UP") {
        moveUp();
    } else if (msg == "DOWN") {
        moveDown();
    } else if (msg == "PAUSE" || msg == "STOP") {
        stopShutter();
    }
    mqttPublishState();
}

// ── Touch handling (polled, debounced) ───────────
void handleTouch() {
    bool upRaw    = digitalRead(PIN_TOUCH_UP);
    bool pauseRaw = digitalRead(PIN_TOUCH_PAUSE);
    bool downRaw  = digitalRead(PIN_TOUCH_DOWN);
    unsigned long now = millis();

    // Debounce: only act when stable for DEBOUNCE_MS
    if (upRaw != lastUpRaw || pauseRaw != lastPauseRaw || downRaw != lastDownRaw) {
        lastDebounceTime = now;
        lastUpRaw = upRaw;
        lastPauseRaw = pauseRaw;
        lastDownRaw = downRaw;
    }

    if ((now - lastDebounceTime) > DEBOUNCE_MS) {
        if (upRaw && !upPressed) {
            upPressed = true;
            Serial.println("[TOUCH] Up");
            moveUp();
        } else if (!upRaw) {
            upPressed = false;
        }

        if (downRaw && !downPressed) {
            downPressed = true;
            Serial.println("[TOUCH] Down");
            moveDown();
        } else if (!downRaw) {
            downPressed = false;
        }

        if (pauseRaw && !pausePressed) {
            pausePressed = true;
            Serial.println("[TOUCH] Pause");
            stopShutter();
        } else if (!pauseRaw) {
            pausePressed = false;
        }
    }
}

// ── End-course detection via current ─────────────
// A running motor already draws current, so we can't just compare against
// idle. We establish a steady "normal running" baseline and treat a sustained
// spike well above that baseline as reaching the mechanical end of travel.
void checkStall() {
    if (shutterState == IDLE) {
        moveBaseline = 0.0f;
        baselineSamples = 0;
        stallStart = 0;
        return;
    }

    float delta = adcAvg - adcOffset;   // current draw above idle
    if (delta <= STALL_THRESHOLD) {
        // Not drawing meaningful current → not running (yet)
        stallStart = 0;
        return;
    }

    // Establish a steady "normal running" baseline over the first few samples,
    // then track it slowly. Do not adapt the baseline during a suspected stall.
    if (baselineSamples < BASELINE_SAMPLES) {
        baselineSamples++;
        moveBaseline = (moveBaseline * (baselineSamples - 1) + delta) / baselineSamples;
        return;
    }
    if (delta <= moveBaseline * STALL_MULTIPLIER) {
        moveBaseline = moveBaseline * 0.95f + delta * 0.05f;
    }

    // End-of-course = current well above the steady running level, sustained
    if (delta > moveBaseline * STALL_MULTIPLIER) {
        if (stallStart == 0) {
            stallStart = millis();
        } else if (millis() - stallStart >= STALL_HOLD_MS) {
            Serial.println("[SHUTTER] End-of-course detected (current spike)");
            // Update tracked position in RAM (no EEPROM write)
            position = (shutterState == MOVING_UP) ? 100 : 0;
            positionValid = true;
            stopShutter();   // publishes state (position now updated)
        }
    } else {
        stallStart = 0;
    }
}

// ── WiFi setup ───────────────────────────────────
void setupWiFi() {
    WiFiManager wm;
    wm.setConfigPortalBlocking(true);
    wm.setDebugOutput(true);

    String apName = "ESP-Shutter-" + String((uint32_t)ESP.getChipId(), HEX);

    bool res = wm.autoConnect(apName.c_str());
    if (!res) {
        Serial.println("[WiFi] Failed to connect, restarting...");
        delay(3000);
        ESP.restart();
    }

    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

// ── mDNS setup ───────────────────────────────────
void setupMDNS() {
    uint32_t chipId = ESP.getChipId();
    String hostname = "esp-shutter-" + String(chipId & 0xFFFF, HEX);

    if (MDNS.begin(hostname.c_str())) {
        Serial.printf("[mDNS] Advertised as http://%s.local\n", hostname.c_str());
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("[mDNS] Failed to start mDNS responder");
    }
}

// ── Web Server ───────────────────────────────────
void setupWebServer() {
    // GET / → help text
    server.on("/", HTTP_GET, []() {
        String help = "ESP-02S Shutter Controller REST API\n\n";
        help += "GET  /state            → shutter state, current, position\n";
        help += "POST /move             → {\"direction\":\"up\"|\"down\"|\"stop\"}\n";
        help += "POST /pause            → stop shutter\n";
        help += "POST /home             → {\"position\":0|100} re-home after reboot (RAM only)\n";
        help += "POST /calibrate        → capture ADC zero-current offset\n";
        help += "GET  /reset/wifi       → reset WiFi and restart\n";
        help += "POST /config/mqtt      → {\"server\":\"\",\"port\":1883,\"topic\":\"\",\"state_topic\":\"\"}\n";
        help += "\nmDNS: http://esp-shutter-" + String((uint32_t)ESP.getChipId() & 0xFFFF, HEX) + ".local\n";
        sendJSON(200, help);
    });

    // GET /state → shutter state
    server.on("/state", HTTP_GET, []() {
        StaticJsonDocument<256> doc;
        doc["direction"] = directionName();
        doc["current"]   = adcToCurrent(adcAvg);
        doc["adc"]       = (int)adcAvg;
        doc["position"]  = position;
        doc["valid"]     = positionValid;

        String out;
        serializeJson(doc, out);
        sendJSON(200, out);
    });

    // POST /move → move shutter
    server.on("/move", HTTP_POST, []() {
        String dir;
        if (server.hasArg("plain")) {
            StaticJsonDocument<128> doc;
            DeserializationError err = deserializeJson(doc, server.arg("plain"));
            if (!err) {
                dir = doc["direction"] | "";
            }
        }
        if (dir.length() == 0 && server.hasArg("dir")) {
            dir = server.arg("dir");
        }
        dir.toLowerCase();

        if (dir == "up") {
            moveUp();
        } else if (dir == "down") {
            moveDown();
        } else if (dir == "stop") {
            stopShutter();
        } else {
            sendJSON(400, "{\"error\":\"invalid direction (up|down|stop)\"}");
            return;
        }

        StaticJsonDocument<128> doc;
        doc["direction"] = directionName();
        String out;
        serializeJson(doc, out);
        sendJSON(200, out);
        mqttPublishState();
    });

    // POST /pause → stop shutter
    server.on("/pause", HTTP_POST, []() {
        stopShutter();
        sendJSON(200, "{\"direction\":\"idle\"}");
        mqttPublishState();
    });

    // POST /home → set known position
    server.on("/home", HTTP_POST, []() {
        int pos = -1;
        if (server.hasArg("plain")) {
            StaticJsonDocument<128> doc;
            DeserializationError err = deserializeJson(doc, server.arg("plain"));
            if (!err) {
                pos = doc["position"] | -1;
            }
        }
        if (pos < 0 && server.hasArg("position")) {
            pos = server.arg("position").toInt();
        }
        if (pos < 0 || pos > 100) {
            sendJSON(400, "{\"error\":\"position must be 0-100\"}");
            return;
        }
        position = pos;
        positionValid = true;
        sendJSON(200, "{\"position\":" + String(position) + ",\"valid\":true}");
        mqttPublishState();
    });

    // POST /calibrate → capture ADC zero-current offset
    server.on("/calibrate", HTTP_POST, []() {
        // Ensure motor is stopped before calibrating
        stopShutter();
        delay(100);
        adcOffset = readAdcFiltered();
        saveAdcOffset(adcOffset);
        Serial.printf("[CALIBRATE] ADC offset = %.1f\n", adcOffset);
        sendJSON(200, "{\"adc_offset\":" + String(adcOffset) + "}");
    });

    // POST /config/mqtt → configure MQTT
    server.on("/config/mqtt", HTTP_POST, []() {
        if (!server.hasArg("plain")) {
            sendJSON(400, "{\"error\":\"missing body\"}");
            return;
        }

        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, server.arg("plain"));
        if (err) {
            sendJSON(400, "{\"error\":\"bad json\"}");
            return;
        }

        const char* srv        = doc["server"] | "";
        int         port       = doc["port"]   | 1883;
        const char* topic      = doc["topic"]  | "";
        const char* stateTopic = doc["state_topic"] | "";

        mqttServer     = srv;
        mqttPort       = port;
        mqttTopic      = topic;
        mqttStateTopic = stateTopic;

        saveMqttConfig(mqttServer, mqttPort, mqttTopic, mqttStateTopic);

        if (mqttServer.length() > 0) {
            mqtt.setServer(mqttServer.c_str(), mqttPort);
            mqttReconnect();
        }

        sendJSON(200, "{\"ok\":true}");
    });

    // GET /reset/wifi → reset WiFi and restart
    server.on("/reset/wifi", HTTP_GET, []() {
        sendJSON(200, "{\"message\":\"Resetting WiFi and restarting...\"}");
        delay(200);
        WiFiManager wm;
        wm.resetSettings();
        ESP.restart();
    });

    // CORS preflight
    server.onNotFound([]() {
        if (server.method() == HTTP_OPTIONS) {
            setCORSHeaders();
            server.send(200, "text/plain", "");
        } else {
            sendJSON(404, "{\"error\":\"Not found\"}");
        }
    });

    server.begin();
    Serial.println("[HTTP] Server started");
}

// ── Setup ────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n=== ESP-02S Shutter Controller ===");

    // GPIOs
    pinMode(PIN_RELAY_ENABLE, OUTPUT);
    pinMode(PIN_RELAY_DIR, OUTPUT);
    digitalWrite(PIN_RELAY_ENABLE, LOW);
    digitalWrite(PIN_RELAY_DIR, LOW);

    pinMode(PIN_TOUCH_UP, INPUT);
    pinMode(PIN_TOUCH_PAUSE, INPUT);
    pinMode(PIN_TOUCH_DOWN, INPUT);

    // EEPROM (initialized once for the whole runtime)
    EEPROM.begin(512);

    // Load persisted config
    loadMqttConfig();

    // Initial ADC reading (zero-current offset if not calibrated)
    adcAvg = readAdcFiltered();
    if (adcOffset == 0.0f) {
        adcOffset = adcAvg;
        saveAdcOffset(adcOffset);
        Serial.printf("[ADC] Initial offset set to %.1f\n", adcOffset);
    }

    // WiFi
    setupWiFi();

    // mDNS
    setupMDNS();

    // MQTT callback
    mqtt.setCallback(mqttCallback);

    // Web server
    setupWebServer();
}

// ── Loop ─────────────────────────────────────────
void loop() {
    server.handleClient();
    MDNS.update();
    mqttLoop();

    // Poll touch sensors (debounced)
    static unsigned long lastPoll = 0;
    unsigned long now = millis();
    if (now - lastPoll >= 10) {
        lastPoll = now;
        handleTouch();
    }

    // Sample ADC periodically and check for end-of-course
    static unsigned long lastAdc = 0;
    if (now - lastAdc >= 50) {
        lastAdc = now;
        adcAvg = readAdcFiltered();
        checkStall();
    }

}
