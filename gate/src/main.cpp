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
#include <Adafruit_AHTX0.h> 
#include <Adafruit_BMP280.h>

#define PIN_RELAY_TOGGLE     2

const unsigned long RELAY_PULSE_MS = 500; //ms

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences prefs;

String mqttServer = "";
uint16_t mqttPort = 1883;
String mqttTopic = "gate/state";
uint16_t mqttDelay = 30; // milli

unsigned long lastMqttPublish = 0;

WiFiManager wm;
AsyncWebServer server(80);

Adafruit_AHTX0 aht; 
Adafruit_BMP280 bmp;

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);

  wm.setConfigPortalBlocking(true);
  wm.setDebugOutput(true);

  bool res = wm.autoConnect("GateConfig", "GateConfig123");

  if (!res) {
    delay(3000);
    ESP.restart();
  }
}

void loadMqttConfig() {
  mqttServer = prefs.getString("server", "");
  mqttPort   = prefs.getUShort("port", 1883);
  mqttTopic  = prefs.getString("topic", "gate/state");
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

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);
      
      JsonDocument resp;
      resp["hum"] = String(humidity.relative_humidity);
      resp["aht_temp"] = String(temp.temperature);
      resp["bmp_temp"] = String(bmp.readTemperature());
      resp["pressure"] = String(bmp.readPressure() / 100.0F);

      String out; 
      serializeJson(resp, out);
      request->send(200, "application/json", out);
  });

  server.on("/config/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
      JsonDocument resp; 
      resp["server"] = mqttServer; 
      resp["port"] = mqttPort; 
      resp["topic"] = mqttTopic; 
      resp["delay"] = mqttDelay;
      String out; 
      serializeJson(resp, out);
      request->send(200, "application/json", out);
    }
  );

  // {"server":"192.168.1.10","port":1883,"topic":"gate/state"}
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
      mqttDelay  = doc["delay"].as<uint16_t>();

      prefs.putString("server", mqttServer);
      prefs.putUShort("port", mqttPort);
      prefs.putString("topic", mqttTopic);
      prefs.putUShort("delay", mqttDelay);

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
    request->send(200, "text/plain", "Available api: GET /reset/wifi ; POST /state/toggle; GET /state ; POST /config/mqtt {'server':'192.168.1.10','port':1883,'topic':'gate/state'}");
  });

  server.begin();
}

void ensureMqttConnected() {
  if (mqttServer.length() == 0) return;

  if (mqttClient.connected()) return;

  String clientId = "gate-toggle-1";
  if (!mqttClient.connect(clientId.c_str())) {
    delay(60000);
  }
}

void publishMqttState() {
  if (mqttServer.length() == 0) return;
  if (!mqttClient.connected()) {
    ensureMqttConnected();
  }

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float pressure = bmp.readPressure() / 100.0F; // hPa

  String payload = "{";
  payload +=  "\"temp\":\"" + String(temp.temperature) + "\",";
  payload +=  "\"pressure\":\"" + String(pressure) + "\",";
  payload +=  "\"humidity\":\"" + String(humidity.relative_humidity) + "\"";
  payload += "}";

  mqttClient.publish(mqttTopic.c_str(), payload.c_str());
}

void setup() {
  delay(2000);
  
  pinMode(PIN_RELAY_TOGGLE, OUTPUT);
  digitalWrite(PIN_RELAY_TOGGLE, LOW);

  Wire.begin(9, 8);

  aht.begin();
  bmp.begin(0x77);

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
    return;
  }

  if (!mqttClient.connected()) {
    ensureMqttConnected();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastMqttPublish >= mqttDelay) {
    publishMqttState();
    lastMqttPublish = now;
  } 
}