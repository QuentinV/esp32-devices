#include <Arduino.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>

// ── Pin configuration ────────────────────────────
#define PIN_LED  2   // onboard LED (active LOW on ESP-02S)

// ── Globals ──────────────────────────────────────
ESP8266WebServer server(80);
RemoteDebug      Debug;   // telnet remote debugger (port 23)

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

// ── WiFi setup ───────────────────────────────────
void setupWiFi() {
    WiFiManager wm;
    wm.setConfigPortalBlocking(true);
    wm.setDebugOutput(true);

    // Build unique AP name from MAC
    String apName = "ESP-Starter-" + String((uint32_t)ESP.getChipId(), HEX);

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
    // Unique hostname from last 4 hex chars of chip ID
    uint32_t chipId = ESP.getChipId();
    String hostname = "esp-starter-" + String(chipId & 0xFFFF, HEX);

    if (MDNS.begin(hostname.c_str())) {
        Serial.printf("[mDNS] Advertised as http://%s.local\n", hostname.c_str());
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("[mDNS] Failed to start mDNS responder");
    }
}

// ── OTA setup (ArduinoOTA) ───────────────────────
// Allows flashing over WiFi from PlatformIO or the Arduino IDE.
//   PlatformIO:  pio run -e starter_esp02s_ota -t upload --upload-port <ip>
void setupOTA() {
    // Unique hostname from last 4 hex chars of chip ID
    uint32_t chipId = ESP.getChipId();
    String hostname = "esp-starter-" + String(chipId & 0xFFFF, HEX);
    ArduinoOTA.setHostname(hostname.c_str());

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
    Debug.begin("esp-starter");   // hostname shown in the telnet banner
    Debug.setResetCmdEnabled(true);  // allow "reset" command from telnet
    Debug.setSerialEnabled(true);    // mirror output to serial monitor
    Serial.println("[DEBUG] RemoteDebug ready on port 23 (telnet)");
}

// ── Web Server ───────────────────────────────────
void setupWebServer() {
    // GET / → help text
    server.on("/", HTTP_GET, []() {
        String help = "ESP-02S Starter Template\n\n";
        help += "GET  /status        → device status (IP, uptime, heap)\n";
        help += "GET  /reset/wifi    → reset WiFi and restart\n";
        help += "\nOTA: flash over WiFi via ArduinoOTA (see README)\n";
        help += "Debug: telnet to this device on port 23\n";
        help += "\nmDNS: http://esp-starter-" + String((uint32_t)ESP.getChipId() & 0xFFFF, HEX) + ".local\n";
        sendJSON(200, help);
    });

    // GET /status → device status
    server.on("/status", HTTP_GET, []() {
        StaticJsonDocument<256> doc;
        doc["ip"]      = WiFi.localIP().toString();
        doc["mac"]     = WiFi.macAddress();
        doc["rssi"]    = WiFi.RSSI();
        doc["uptime_s"] = millis() / 1000;
        doc["heap"]    = ESP.getFreeHeap();
        doc["chip_id"] = String((uint32_t)ESP.getChipId(), HEX);

        String out;
        serializeJson(doc, out);
        sendJSON(200, out);
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

    Serial.println("\n\n=== ESP-02S Starter Template ===");

    // GPIOs
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);   // LED off (active LOW)

    // WiFi
    setupWiFi();

    // mDNS
    setupMDNS();

    // OTA
    setupOTA();

    // Remote debug (telnet)
    setupRemoteDebug();

    // Web server
    setupWebServer();
}

// ── Loop ─────────────────────────────────────────
void loop() {
    server.handleClient();
    MDNS.update();
    ArduinoOTA.handle();
    Debug.handle();

    // Blink the onboard LED to show the device is alive
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long now = millis();
    if (now - lastBlink >= 1000) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState ? LOW : HIGH);   // active LOW
        Debug.printf("[LED] %s\n", ledState ? "ON" : "OFF");
    }
}