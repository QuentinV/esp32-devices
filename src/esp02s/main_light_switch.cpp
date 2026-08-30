#include <Arduino.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>

// ── Pin configuration ────────────────────────────
#define PIN_RELAY   12   // relay (light)
#define PIN_SWITCH  4   // physical switch (GND when pressed)

// ── Physical switch debounce ─────────────────────
#define DEBOUNCE_MS   50
#define MQTT_EEPROM_ADDR 0

// ── Globals ──────────────────────────────────────
ESP8266WebServer server(80);
WiFiClient       espClient;
PubSubClient     mqtt(espClient);
RemoteDebug      Debug;   // telnet remote debugger (port 23)
String           deviceKey;

// Lights state
bool relayState   = false;     // current relay state
bool lastSwitchState  = false;     // last stable switch reading

// MQTT config (persisted in EEPROM)
String mqttServer;
int    mqttPort   = 1883;
String mqttTopic;
String mqttClientId;

// Debounce
bool    lastSwitchRaw = HIGH;   // previous raw reading (for edge/debounce detection)
unsigned long lastDebounceTime = 0;

// MQTT reconnect throttle
unsigned long lastMqttAttempt = 0;

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

// ── Relay control ────────────────────────────────
void setRelay(bool on) {
    relayState = on;
    digitalWrite(PIN_RELAY, on ? HIGH : LOW);
    Serial.printf("[RELAY] Light %s\n", on ? "ON" : "OFF");
    Debug.printf("[RELAY] Light %s\n", on ? "ON" : "OFF");
}

void toggleRelay() {
    setRelay(!relayState);
}

// ── MQTT ─────────────────────────────────────────
void loadMqttConfig() {
    EEPROM.begin(512);
    // Simple strategy: store as JSON in EEPROM at offset 0
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
            mqttServer   = doc["server"] | "";
            mqttPort     = doc["port"]   | 1883;
            mqttTopic    = doc["topic"]  | "";
            mqttClientId = doc["client"] | "";
        }
    }

    // Fallback client ID
    if (mqttClientId.length() == 0) {
        mqttClientId = deviceKey;
    }

    if (mqttServer.length() > 0) {
        mqtt.setServer(mqttServer.c_str(), mqttPort);
        Serial.printf("[MQTT] Config loaded: %s:%d topic=%s\n",
            mqttServer.c_str(), mqttPort, mqttTopic.c_str());
    }
}

void saveMqttConfig(const String& srv, int port, const String& topic) {
    StaticJsonDocument<256> doc;
    doc["server"] = srv;
    doc["port"]   = port;
    doc["topic"]  = topic;
    doc["client"] = mqttClientId;

    String json;
    serializeJson(doc, json);

    EEPROM.begin(512);
    for (size_t i = 0; i < json.length(); i++) {
        EEPROM.write(MQTT_EEPROM_ADDR + i, json[i]);
    }
    EEPROM.write(MQTT_EEPROM_ADDR + json.length(), '\0');
    EEPROM.commit();
}

void mqttPublishState() {
    if (mqttServer.length() == 0) return;
    if (!mqtt.connected()) return;

    String power = relayState ? "ON" : "OFF";
    String payload = "{\"externalId\":\"" + deviceKey + "\",\"power\":\"" + power + "\"}";
    if (mqttTopic.length() > 0) {
        mqtt.publish(mqttTopic.c_str(), payload.c_str());
        Serial.printf("[MQTT] Published %s to %s\n", payload.c_str(), mqttTopic.c_str());
    }
}

void mqttReconnect() {
    if (mqttServer.length() == 0) return;

    Serial.printf("[MQTT] Connecting to %s:%d as %s ...\n",
        mqttServer.c_str(), mqttPort, mqttClientId.c_str());

    if (mqtt.connect(mqttClientId.c_str())) {
        Serial.println("[MQTT] Connected");
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

// ── Physical switch handling ─────────────────────
void handleSwitchInterrupt() {
    bool raw = digitalRead(PIN_SWITCH);
    unsigned long now = millis();

    if (raw != lastSwitchRaw) {
        lastSwitchRaw = raw;
        lastDebounceTime = now;
    }

    // Two-position interruptor: any stable change of position is one toggle
    // event, regardless of which position it moves to. lastSwitchState is the
    // position captured at boot (or after all blocking setup), so powering
    // up in either position never triggers — only changes made *after*
    // startup are acted on.
    if ((now - lastDebounceTime) > DEBOUNCE_MS && raw != lastSwitchState) {
        lastSwitchState = raw;
        toggleRelay();
        mqttPublishState();
    }
}

// ── WiFi setup ───────────────────────────────────
void setupWiFi() {
    WiFiManager wm;
    wm.setConfigPortalBlocking(true);
    wm.setDebugOutput(true);

    // Build unique AP name from MAC
    bool res = wm.autoConnect(deviceKey.c_str());
    if (!res) {
        Serial.println("[WiFi] Failed to connect, restarting...");
        delay(3000);
        ESP.restart();
    }

    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

// ── mDNS setup ───────────────────────────────────
void setupMDNS() {
    String hostname = deviceKey;

    if (MDNS.begin(hostname.c_str())) {
        Serial.printf("[mDNS] Advertised as http://%s.local\n", hostname.c_str());
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("[mDNS] Failed to start mDNS responder");
    }
}

// ── OTA setup (ArduinoOTA) ───────────────────────
// Allows flashing over WiFi from PlatformIO or the Arduino IDE.
//   PlatformIO:  pio run -e light_switch_esp02s_ota -t upload --upload-port <ip>
void setupOTA() {
    ArduinoOTA.setHostname(deviceKey.c_str());

    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] Start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)      Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)   Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("[OTA] Ready (ArduinoOTA)");
}

// ── Remote debug setup (telnet) ─────────────────
// Connect with any telnet client:  telnet <ip> 23
// Commands: help, debug level, etc. (type "help" once connected)
void setupRemoteDebug() {
    Debug.begin("esp-light");   // hostname shown in the telnet banner
    Debug.setResetCmdEnabled(true);  // allow "reset" command from telnet
    Debug.setSerialEnabled(true);    // mirror output to serial monitor
    Serial.println("[DEBUG] RemoteDebug ready on port 23 (telnet)");
}

void setupWebServer() {
    // GET / → help text
    server.on("/", HTTP_GET, []() {
        String help = "ESP Light Switch REST API\n\n";
        help += "GET  /             → this help\n";
        help += "GET  /state   → light state\n";
        help += "POST /toggle?set=0|1 → toggle or set light\n";
        help += "GET  /reset/wifi   → reset WiFi and restart\n";
        help += "POST /config/mqtt  → {\"server\":\"\",\"port\":1883,\"topic\":\"\"}\n";
        help += "\nmDNS: http://" + deviceKey + ".local\n";
        sendJSON(200, help);
    });

    // GET /state → get light state
    server.on("/state", HTTP_GET, []() {
        String state = relayState ? "high" : "low";
        sendJSON(200, "{\"state\":\"" + state + "\"}");
    });

    // POST /toggle?set=0|1 → toggle or set light
    server.on("/toggle", HTTP_POST, []() {
        if (server.hasArg("set")) {
            int target = server.arg("set").toInt();
            setRelay(target == 1);
        } else {
            toggleRelay();
        }

        String state = relayState ? "high" : "low";
        sendJSON(200, "{\"state\":\"" + state + "\"}");

        mqttPublishState();
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

        const char* srv   = doc["server"] | "";
        int         port  = doc["port"]   | 1883;
        const char* topic = doc["topic"]  | "";

        mqttServer = srv;
        mqttPort   = port;
        mqttTopic  = topic;

        saveMqttConfig(mqttServer, mqttPort, mqttTopic);

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

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n=== ESP-02S Light Switch ===");
    deviceKey = "esp-light-" + String(ESP.getChipId() & 0xFFFF, HEX);
    Serial.println(deviceKey);
    
    // GPIOs
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);
    relayState = false;

    pinMode(PIN_SWITCH, INPUT_PULLUP);
    lastSwitchRaw   = digitalRead(PIN_SWITCH);
    lastSwitchState = lastSwitchRaw;

    loadMqttConfig();
    setupWiFi();
    setupMDNS();
    setupOTA();
    setupRemoteDebug();
    setupWebServer();
 
    lastSwitchRaw     = digitalRead(PIN_SWITCH);
    lastSwitchState   = lastSwitchRaw;
    lastDebounceTime  = millis();
}

void loop() {
    server.handleClient();
    MDNS.update();
    ArduinoOTA.handle();
    Debug.handle();
    mqttLoop();

    // Poll physical switch (using poll instead of interrupt for simplicity/robustness)
    // Using a short block so we don't miss edge changes. On ESP8266, interrupts on
    // simple GPIO toggles can be unreliable with WiFi; polling is safer.
    static unsigned long lastPoll = 0;
    unsigned long now = millis();
    if (now - lastPoll >= 10) {
        lastPoll = now;
        handleSwitchInterrupt();
    }
}