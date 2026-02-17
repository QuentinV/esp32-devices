#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h> 

#define PIN_RELAY_TOGGLE     6
#define PIN_MOTION_SENSOR    5

#define MQTT_CLIENT_ID "garagedoor-toggle-1"

const unsigned long RELAY_PULSE_MS = 500; //ms

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences prefs;

String mqttServer = "";
uint16_t mqttPort = 1883;
String mqttTopic = "";

int lastState = 0;

WiFiManager wm;
AsyncWebServer server(80);

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);

  wm.setConfigPortalBlocking(true);
  wm.setDebugOutput(true);

  bool res = wm.autoConnect("GarageDoorConfig", "GarageDoorConfig123");

  if (!res) {
    delay(3000);
    ESP.restart();
  }
}

void loadMqttConfig() {
  mqttServer = prefs.getString("server", "");
  mqttPort   = prefs.getUShort("port", 1883);
  mqttTopic  = prefs.getString("topic", "garagedoor/state");
}

void setupWebServer() {
  auto sendJson = [](AsyncWebServerRequest *request, int code, const String &body) {
    request->send(code, "application/json", body);
  };

  server.on("/state/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {
      digitalWrite(PIN_RELAY_TOGGLE, HIGH);
      delay(RELAY_PULSE_MS);
      digitalWrite(PIN_RELAY_TOGGLE, LOW);
      request->send(200, "application/text", "");
    }
  );

  server.on("/config/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
      JsonDocument resp; 
      resp["server"] = mqttServer; 
      resp["port"] = mqttPort; 
      resp["topic"] = mqttTopic;
      String out; 
      serializeJson(resp, out);
      request->send(200, "application/json", out);
    }
  );

  // {"server":"192.168.1.10","port":1883,"topic":"garagedoor/state"}
  server.on("/config/mqtt", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);

      if (err) {
        request->send(400, "application/json", "{\"error\":\"invalid JSON\"}");
        return;
      }

      if (!doc.containsKey("server") ||
          !doc.containsKey("port") ||
          !doc.containsKey("topic")) {
        request->send(400, "application/json", "{\"error\":\"missing fields\"}");
        return;
      }

      mqttServer = doc["server"].as<String>();
      mqttPort   = doc["port"].as<uint16_t>();
      mqttTopic  = doc["topic"].as<String>();

      prefs.putString("server", mqttServer);
      prefs.putUShort("port", mqttPort);
      prefs.putString("topic", mqttTopic);

      mqttClient.setServer(mqttServer.c_str(), mqttPort);

      request->send(200, "application/text", "");
    }
  );
  
  server.on("/reset/wifi", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(200, "text/plain", "Ok");
      wm.resetSettings();
      ESP.restart();
    }
  );

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Available api: GET /reset/wifi ; POST /state/toggle; POST /config/mqtt {'server':'192.168.1.10','port':1883,'topic':'garagedoor/state'}");
  });

  server.begin();
}

void ensureMqttConnected() {
  if (mqttServer.length() == 0) return;

  if (mqttClient.connected()) return;

  String clientId = MQTT_CLIENT_ID;
  if (!mqttClient.connect(clientId.c_str())) {
    delay(60000);
  }
}

void publishMqttState(boolean value) {
  if (mqttServer.length() == 0) return;
  if (!mqttClient.connected()) {
    ensureMqttConnected();
  }

  //Serial.print("publish state...");

  String deviceId = MQTT_CLIENT_ID;
  String payload = "{";
  payload +=  "\"deviceId\":\"" + deviceId + "\",";
  payload +=  "\"value\":\"" + String(value) + "\"";
  payload += "}";

  mqttClient.publish(mqttTopic.c_str(), payload.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  
  pinMode(PIN_RELAY_TOGGLE, OUTPUT);
  digitalWrite(PIN_RELAY_TOGGLE, LOW);

  pinMode(PIN_MOTION_SENSOR, INPUT_PULLDOWN);
  
  prefs.begin("mqtt", false);
  loadMqttConfig();

  
  setupWiFi();

  if (mqttServer.length() > 0) {
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
  }

  setupWebServer();  
}

void loop() {
  if (mqttServer.length() == 0) {
    delay(60000);
    return;
  }

  if (!mqttClient.connected()) {
    ensureMqttConnected();
  }
  mqttClient.loop();

  int read = digitalRead(PIN_MOTION_SENSOR);
  //Serial.print("reading motion sensor");
  if (lastState != read) {
      lastState = read;
      publishMqttState(read == 1);
  }  

  delay(50);
}