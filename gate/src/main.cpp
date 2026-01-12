#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESPAsyncWiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h> 
#include <Adafruit_AHTX0.h> 
#include <Adafruit_BMP280.h>

#define PIN_RELAY_OPEN       2
#define PIN_RELAY_CLOSE      3

const unsigned long RELAY_PULSE_MS = 500; //ms

WiFiClient espClient;
PubSubClient mqttClient(espClient);

String mqttServer = "";
uint16_t mqttPort = 1883;
String mqttTopic = "gate/state";
uint16_t mqttDelay = 30; // milli

unsigned long lastMqttPublish = 0;

AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns);
String state = "closed";

Adafruit_AHTX0 aht; 
Adafruit_BMP280 bmp;

bool loadConfig() {
  if (!LittleFS.exists("/config.json")) return false;

  File f = LittleFS.open("/config.json", "r");
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return false;

  mqttServer = doc["mqttServer"] | "";
  mqttPort   = doc["mqttPort"]   | 1883;
  mqttTopic  = doc["mqttTopic"]  | "gate/state";

  Serial.println("Loaded MQTT config:");
  Serial.print("  server: "); Serial.println(mqttServer);
  Serial.print("  port  : "); Serial.println(mqttPort);
  Serial.print("  topic : "); Serial.println(mqttTopic);

  return true;
}

void saveConfig(String server, uint16_t port, String topic) {
  DynamicJsonDocument doc(256);
  doc["mqttServer"] = server;
  doc["mqttPort"]   = port;
  doc["mqttTopic"]  = topic;

  File f = LittleFS.open("/config.json", "w");
  serializeJson(doc, f);
  f.close();
}

void setupWiFi() {
  bool res = wifiManager.autoConnect("GateConfig", "GateConfig12345");
  if (!res) {
    Serial.println("Failed to connect, restarting...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void pulseRelay(int pin) {
  digitalWrite(pin, HIGH);
  delay(RELAY_PULSE_MS);  // this is blocking; you can make it non-blocking later
  digitalWrite(pin, LOW);
}

void setupWebServer() {
  auto sendJson = [](AsyncWebServerRequest *request, int code, const String &body) {
    request->send(code, "application/json", body);
  };

  // ---- Sned command ----
  // { "command": "open" | "close" | "toggle" | "pedestrian" }
  server.on("/state", HTTP_POST,
    [](AsyncWebServerRequest *request){},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {

      // Parse JSON
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);

      if (err) {
        request->send(400, "application/json", "{\"error\":\"invalid JSON\"}");
        return;
      }

      if (!doc.containsKey("command")) {
        request->send(400, "application/json", "{\"error\":\"missing command\"}");
        return;
      }

      String command = doc["command"].as<String>();
      command.toLowerCase();

      if (command == "open") {
        //pulseRelay(PIN_RELAY_OPEN);
        state = "opened";
      } else if (command == "close") {
        //pulseRelay(PIN_RELAY_CLOSE);
        state = "closed";
      } else {
        request->send(400, "application/json", "{\"error\":\"unknown command\"}");
        return;
      }

      request->send(204);
    }
  );

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);
      
      JsonDocument resp; 
      resp["state"] = state;
      resp["hum"] = String(humidity.relative_humidity);
      resp["aht_temp"] = String(temp.temperature);
      resp["bmp_temp"] = String(bmp.readTemperature());
      resp["pressure"] = String(bmp.readPressure() / 100.0F);

      String out; 
      serializeJson(resp, out);
      request->send(200, "application/json", out);
  });

  // ---- MQTT config ----
  // {"server":"192.168.1.10","port":1883,"topic":"gate/state"}
  server.on("/config/mqtt", HTTP_POST,
    [](AsyncWebServerRequest *request){},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {

      // Parse JSON body
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);

      if (err) {
        request->send(400, "application/json", "{\"error\":\"invalid JSON\"}");
        return;
      }

      // Validate required fields
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

      /*prefs.putString("server", mqttServer);
      prefs.putUShort("port", mqttPort);
      prefs.putString("topic", mqttTopic);
      prefs.putUShort("delay", mqttDelay);*/

      mqttClient.setServer(mqttServer.c_str(), mqttPort);

      request->send(204);
    }
  );

  server.on("/config/mqtt", HTTP_GET,
    [](AsyncWebServerRequest *request){},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
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

  server.on("/reset/wifi", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(200, "text/plain", "Ok");
      wifiManager.resetSettings();
      ESP.restart();
    }
  );

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Available api: GET /reset/wifi ; POST /state { 'command': 'open' | 'close' } ; GET /state ; POST /config/mqtt {'server':'192.168.1.10','port':1883,'topic':'gate/state'}");
  });

  server.begin();
  Serial.println("HTTP server started.");
}

/*
void ensureMqttConnected() {
  if (mqttServer.length() == 0) return;

  if (mqttClient.connected()) return;

  Serial.print("Connecting to MQTT: ");
  Serial.print(mqttServer);
  Serial.print(":");
  Serial.println(mqttPort);

  String clientId = "gate-";
  clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT connected.");
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
    delay(1000);
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
  payload += "\"state\":\"" + state + "\",";
  payload +=  "\"temp\":\"" + String(temp.temperature) + "\",";
  payload +=  "\"pressure\":\"" + String(pressure) + "\",";
  payload +=  "\"humidity\":\"" + String(humidity.relative_humidity) + "\"";
  payload += "}";

  mqttClient.publish(mqttTopic.c_str(), payload.c_str());
}
*/

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("\nBooting...");

  // Start filesystem
  //if (!LittleFS.begin()) {
  //  Serial.println("LittleFS mount failed");
  //}

  Wire.begin(D2, D1);

  aht.begin();
  bmp.begin(0x77);
  
  /*// Configure relay pins
  pinMode(PIN_RELAY_OPEN, OUTPUT);
  pinMode(PIN_RELAY_CLOSE, OUTPUT);

  digitalWrite(PIN_RELAY_OPEN, LOW);
  digitalWrite(PIN_RELAY_CLOSE, LOW);
  */

  // MQTT config
  //prefs.begin("mqtt", false);
  //loadMqttConfig();

  // WiFi via WiFiManager
  setupWiFi();

  // MQTT client setup
  //if (mqttServer.length() > 0) {
  //  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  //}

  setupWebServer();
}

void loop() {
  // Non-blocking main loop; AsyncWebServer handles HTTP in background

 /* if (mqttServer.length() == 0) {
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
  }  */

  /*sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float pressure = bmp.readPressure() / 100.0F; // hPa

  String payload = "{";
  payload += "\"state\":\"" + state + "\",";
  payload +=  "\"temp\":\"" + String(temp.temperature) + "\",";
  payload +=  "\"pressure\":\"" + String(pressure) + "\",";
  payload +=  "\"humidity\":\"" + String(humidity.relative_humidity) + "\"";
  payload += "}";

  Serial.println(payload);

  delay(1000);*/
 
}