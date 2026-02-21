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
#include <Adafruit_NeoPixel.h>

#define PIN_RELAY_TOGGLE     0

#define MQTT_CLIENT_ID "gate2-toggle-1"

const unsigned long RELAY_PULSE_MS = 500; //ms

WiFiClient espClient;
Preferences prefs;

WiFiManager wm;
AsyncWebServer server(80);


Adafruit_NeoPixel pixels(1, GPIO_NUM_10, NEO_GRB + NEO_KHZ800);

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_max_tx_power(30);

  wm.setConfigPortalBlocking(true);
  wm.setDebugOutput(true);

  bool res = wm.autoConnect("GarageDoorConfig", "GarageDoorConfig123");

  if (!res) {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    delay(3000);
    ESP.restart();
  }
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

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
}


void setup() {
  Serial.begin(115200);
  delay(5000);
  
  pinMode(PIN_RELAY_TOGGLE, OUTPUT);
  digitalWrite(PIN_RELAY_TOGGLE, LOW);


  pixels.begin();
  pixels.setBrightness(1);
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  
  setupWiFi();
  
  setupWebServer();  
}

void loop() {
}