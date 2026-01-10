#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define PIN_RELAY_OPEN       2
#define PIN_RELAY_CLOSE      3
#define PIN_RELAY_TOGGLE     4
#define PIN_RELAY_PEDESTRIAN 5

#define DHTPIN GPIO_NUM_14
#define DHTTYPE DHT22

const unsigned long RELAY_PULSE_MS = 500; //ms

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences prefs;

String mqttServer = "";
uint16_t mqttPort = 1883;
String mqttTopic = "gate/state";
uint16_t mqttDelay = 30; // milli

unsigned long lastMqttPublish = 0;

AsyncWebServer server(80);
String state = "closed";

DHT dht(DHTPIN, DHTTYPE);

void setupWiFi() {
  WiFi.mode(WIFI_STA);

  WiFiManager wm;
  wm.setConfigPortalBlocking(true);
  wm.setDebugOutput(true);

  bool res = wm.autoConnect("GateConfig", "GateConfig123");

  if (!res) {
    Serial.println("Failed to connect, restarting...");
    delay(3000);
    ESP.restart();
  } else {
    Serial.print("Connected to WiFi: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
}

void loadMqttConfig() {
  mqttServer = prefs.getString("server", "");
  mqttPort   = prefs.getUShort("port", 1883);
  mqttTopic  = prefs.getString("topic", "gate/state");

  Serial.println("Loaded MQTT config:");
  Serial.print("  server: "); Serial.println(mqttServer);
  Serial.print("  port  : "); Serial.println(mqttPort);
  Serial.print("  topic : "); Serial.println(mqttTopic);
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
        pulseRelay(PIN_RELAY_OPEN);
        state = "opened";
      } else if (command == "close") {
        pulseRelay(PIN_RELAY_CLOSE);
        state = "closed";
      } else if (command == "toggle") {
        pulseRelay(PIN_RELAY_TOGGLE);
        state = state == "opened" ? "closed" : "opened";
      } else if (command == "pedestrian") {
        pulseRelay(PIN_RELAY_PEDESTRIAN);
        state = "pedestrian";
      } else {
        request->send(400, "application/json", "{\"error\":\"unknown command\"}");
        return;
      }

      request->send(204);
    }
  );

  server.on("/state", HTTP_GET, 
    [](AsyncWebServerRequest *request){},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      JsonDocument resp; 
      resp["state"] = state;
      String out; serializeJson(resp, out);
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

      prefs.putString("server", mqttServer);
      prefs.putUShort("port", mqttPort);
      prefs.putString("topic", mqttTopic);
      prefs.putUShort("delay", mqttDelay);

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Available api: POST /state { 'command': 'open' | 'close' | 'toggle' | 'pedestrian' } or POST /config/mqtt {'server':'192.168.1.10','port':1883,'topic':'gate/state'}");
  });

  server.begin();
  Serial.println("HTTP server started.");
}

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

  float temp = dht.readTemperature();
  float hum  = dht.readHumidity(); 

  String payload = "{";
  payload += "\"state\":\"" + state + "\",";
  payload +=  "\"temperature\":\"" + String(temp) + "\",";
  payload +=  "\"humidity\":\"" + String(hum) + "\"";
  payload += "}";

  mqttClient.publish(mqttTopic.c_str(), payload.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBooting...");

  // Configure relay pins
  pinMode(PIN_RELAY_OPEN, OUTPUT);
  pinMode(PIN_RELAY_CLOSE, OUTPUT);
  pinMode(PIN_RELAY_TOGGLE, OUTPUT);
  pinMode(PIN_RELAY_PEDESTRIAN, OUTPUT);

  digitalWrite(PIN_RELAY_OPEN, LOW);
  digitalWrite(PIN_RELAY_CLOSE, LOW);
  digitalWrite(PIN_RELAY_TOGGLE, LOW);
  digitalWrite(PIN_RELAY_PEDESTRIAN, LOW);

  dht.begin();

  // MQTT config
  prefs.begin("mqtt", false);
  loadMqttConfig();

  // WiFi via WiFiManager
  setupWiFi();

  // MQTT client setup
  if (mqttServer.length() > 0) {
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
  }

  // REST API
  setupWebServer();
}

void loop() {
  // Non-blocking main loop; AsyncWebServer handles HTTP in background

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