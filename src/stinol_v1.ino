// ============================================================================
// stinol_v1.ino — Stinol 102L Smart Refrigerator Controller v1 (Simple)
// Monolithic single-file version for easy understanding and quick start
// ============================================================================

#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>

// ===== Pin Configuration =====
#define PIN_ONE_WIRE  D4    // DS18B20
#define PIN_SSR       D1    // Solid-state relay
#define PIN_NEOPIXEL  D2    // WS2812B LED

// ===== Temperature Thresholds =====
float compOnTemp  = 6.0;
float compOffTemp = 3.5;

// ===== Timing (ms) =====
const unsigned long MIN_ON_TIME   = 180000;   // 3 min
const unsigned long MIN_OFF_TIME  = 180000;   // 3 min
const unsigned long MAX_RUNTIME   = 3600000;  // 60 min
const unsigned long READ_INTERVAL = 2000;      // 2 sec

// ===== State =====
bool relayOn = false;
bool sensorOk = false;
float currentTemp = -127.0;
unsigned long relayChangeMs = 0;
unsigned long lastReadMs = 0;
unsigned long compStartMs = 0;
bool wifiConnected = false;

// ===== Objects =====
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature dallas(&oneWire);
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
AsyncMqttClient mqtt;
DNSServer dnsServer;

// ===== WiFi Credentials (loaded from LittleFS) =====
char wifiSsid[65] = "";
char wifiPass[65] = "";
char mqttHost[129] = "";
uint16_t mqttPort = 1883;
char mqttUser[49] = "";
char mqttPass[49] = "";

// ===== Session Auth =====
String sessionToken = "";
unsigned long sessionTime = 0;

// ===== Median Filter =====
float samples[5] = {};
uint8_t sampleIdx = 0;
uint8_t sampleCount = 0;

float medianFilter(float val) {
    samples[sampleIdx] = val;
    sampleIdx = (sampleIdx + 1) % 5;
    if (sampleCount < 5) sampleCount++;
    float sorted[5];
    for (uint8_t i = 0; i < sampleCount; i++) sorted[i] = samples[i];
    // Insertion sort
    for (uint8_t i = 1; i < sampleCount; i++) {
        float key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
        sorted[j+1] = key;
    }
    return sorted[sampleCount / 2];
}

// ===== Settings Load/Save =====
bool loadSettings() {
    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    File f = LittleFS.open("/settings.json", "r");
    if (!f) return false;
    JsonDocument doc;
    if (deserializeJson(doc, f)) { f.close(); return false; }
    f.close();
    if (doc.containsKey("wifiSsid")) strncpy(wifiSsid, doc["wifiSsid"], 64);
    if (doc.containsKey("wifiPass")) strncpy(wifiPass, doc["wifiPass"], 64);
    if (doc.containsKey("mqttHost")) strncpy(mqttHost, doc["mqttHost"], 128);
    mqttPort = doc["mqttPort"] | 1883;
    if (doc.containsKey("mqttUser")) strncpy(mqttUser, doc["mqttUser"], 48);
    if (doc.containsKey("mqttPass")) strncpy(mqttPass, doc["mqttPass"], 48);
    compOnTemp = doc["compOnTemp"] | 6.0;
    compOffTemp = doc["compOffTemp"] | 3.5;
    return true;
}

bool saveSettings() {
    JsonDocument doc;
    doc["wifiSsid"] = wifiSsid;
    doc["wifiPass"] = wifiPass;
    doc["mqttHost"] = mqttHost;
    doc["mqttPort"] = mqttPort;
    doc["mqttUser"] = mqttUser;
    doc["mqttPass"] = mqttPass;
    doc["compOnTemp"] = compOnTemp;
    doc["compOffTemp"] = compOffTemp;
    File f = LittleFS.open("/settings.json", "w");
    if (!f) return false;
    serializeJson(doc, f);
    f.close();
    return true;
}

// ===== Compressor Control =====
void compressorControl() {
    unsigned long now = millis();
    unsigned long elapsed = now - relayChangeMs;

    if (relayOn) {
        // Safety: max runtime
        if (now - compStartMs > MAX_RUNTIME) {
            relayOn = false;
            relayChangeMs = now;
            Serial.println("[COMP] Max runtime exceeded, stopping");
            return;
        }
        // Min on time
        if (elapsed < MIN_ON_TIME) return;
        // Check temperature
        if (sensorOk && currentTemp <= compOffTemp) {
            relayOn = false;
            relayChangeMs = now;
            Serial.printf("[COMP] Stopped at %.1f°C\n", currentTemp);
        }
    } else {
        // Min off time
        if (elapsed < MIN_OFF_TIME) return;
        // Check temperature
        if (sensorOk && currentTemp >= compOnTemp) {
            relayOn = true;
            compStartMs = now;
            relayChangeMs = now;
            Serial.printf("[COMP] Started at %.1f°C\n", currentTemp);
        }
    }

    digitalWrite(PIN_SSR, relayOn ? HIGH : LOW);
}

// ===== Sensor Reading =====
void readSensor() {
    unsigned long now = millis();
    if (now - lastReadMs < READ_INTERVAL) return;
    lastReadMs = now;

    dallas.requestTemperatures();
    float raw = dallas.getTempCByIndex(0);

    if (raw == DEVICE_DISCONNECTED_C || raw < -126.0 || (raw > 84.0 && raw < 86.0)) {
        sensorOk = false;
        return;
    }

    sensorOk = true;
    currentTemp = medianFilter(raw);
}

// ===== LED Status =====
void updateLED() {
    static unsigned long lastLed = 0;
    static bool blinkState = false;
    if (millis() - lastLed < 500) return;
    lastLed = millis();
    blinkState = !blinkState;

    if (!sensorOk) {
        pixel.setPixelColor(0, blinkState ? pixel.Color(255,0,0) : 0);
    } else if (!wifiConnected) {
        pixel.setPixelColor(0, blinkState ? pixel.Color(0,200,255) : 0);
    } else if (relayOn) {
        pixel.setPixelColor(0, pixel.Color(0,200,0));
    } else {
        pixel.setPixelColor(0, pixel.Color(0,50,0));
    }
    pixel.show();
}

// ===== WiFi Auto-Reconnect (persWifiManager approach) =====
unsigned long lastWifiAttempt = 0;
uint16_t wifiRetryDelay = 1000;

void wifiReconnect() {
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        wifiRetryDelay = 1000;
        return;
    }

    wifiConnected = false;
    unsigned long now = millis();
    if (now - lastWifiAttempt < wifiRetryDelay) return;
    lastWifiAttempt = now;

    Serial.println("[WIFI] Reconnecting...");
    WiFi.begin(wifiSsid, wifiPass);
    wifiRetryDelay = min(wifiRetryDelay * 2, (uint16_t)30000);
}

// ===== MQTT =====
unsigned long lastMqttAttempt = 0;
uint16_t mqttRetryDelay = 5000;
unsigned long lastMqttPublish = 0;

void mqttReconnect() {
    if (mqtt.connected() || !wifiConnected || strlen(mqttHost) == 0) return;
    unsigned long now = millis();
    if (now - lastMqttAttempt < mqttRetryDelay) return;
    lastMqttAttempt = now;
    mqtt.connect();
}

void mqttPublishState() {
    if (!mqtt.connected()) return;
    unsigned long now = millis();
    if (now - lastMqttPublish < 30000) return;
    lastMqttPublish = now;

    JsonDocument doc;
    doc["temperature"] = roundf(currentTemp * 100) / 100.0;
    doc["sensor_ok"] = sensorOk;
    doc["relay_on"] = relayOn;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["uptime"] = millis() / 1000;

    String json;
    serializeJson(doc, json);
    mqtt.publish("stinol/state", 0, true, json.c_str());
}

// ===== Web Server =====
void setupWebServer() {
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["temperature"] = roundf(currentTemp * 100) / 100.0;
        doc["sensor_ok"] = sensorOk;
        doc["relay_on"] = relayOn;
        doc["wifi_connected"] = wifiConnected;
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["mqtt_connected"] = mqtt.connected();
        doc["free_heap"] = ESP.getFreeHeap();
        doc["uptime"] = millis() / 1000;
        doc["ip"] = WiFi.localIP().toString();
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    server.on("/api/login", HTTP_POST, [](AsyncWebServerRequest *req) {},
        [](AsyncWebServerRequest *req, const String& f, size_t i, uint8_t* d, size_t l, bool fin) {},
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len) {
            JsonDocument doc;
            deserializeJson(doc, data, len);
            if (strcmp(doc["username"] | "", "admin") == 0 &&
                strcmp(doc["password"] | "", "stinol102") == 0) {
                sessionToken = String(RANDOM_REG32, HEX) + String(RANDOM_REG32, HEX);
                sessionTime = millis();
                JsonDocument r;
                r["token"] = sessionToken;
                String j; serializeJson(r, j);
                req->send(200, "application/json", j);
            } else {
                req->send(403, "application/json", "{\"error\":\"Invalid\"}");
            }
        });

    server.begin();
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Stinol 102L v1 ===\n");

    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);

    dallas.begin();
    dallas.setResolution(12);
    dallas.setWaitForConversion(false);

    pixel.begin();
    pixel.setBrightness(40);

    loadSettings();

    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.mode(WIFI_STA);
    if (strlen(wifiSsid) > 0) {
        WiFi.begin(wifiSsid, wifiPass);
    } else {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP("Stinol-Setup");
        dnsServer.start(53, "*", WiFi.softAPIP());
    }

    mqtt.onConnect([](bool s) {
        Serial.println("[MQTT] Connected!");
        mqtt.subscribe("stinol/command/#", 1);
        mqttRetryDelay = 5000;
    });
    mqtt.onDisconnect([](AsyncMqttClientDisconnectReason r) {
        Serial.printf("[MQTT] Disconnected: %d\n", r);
        mqttRetryDelay = min(mqttRetryDelay * 2, (uint16_t)30000);
    });
    mqtt.onMessage([](char* topic, char* payload,
                       AsyncMqttClientMessageProperties p,
                       size_t len, size_t i, size_t t) {
        char msg[64];
        size_t cl = min(len, sizeof(msg)-1);
        memcpy(msg, payload, cl); msg[cl] = 0;
        // Handle commands here if needed
    });
    if (strlen(mqttHost) > 0) {
        mqtt.setServer(mqttHost, mqttPort);
        if (strlen(mqttUser) > 0) mqtt.setCredentials(mqttUser, mqttPass);
    }
    mqtt.setClientId("stinol102");
    mqtt.setKeepAlive(60);

    setupWebServer();

    ArduinoOTA.setPassword("stinol102");
    ArduinoOTA.begin();

    Serial.println("[SETUP] Complete!");
}

// ===== Main Loop =====
void loop() {
    readSensor();
    compressorControl();
    wifiReconnect();
    mqttReconnect();
    mqttPublishState();
    updateLED();
    ArduinoOTA.handle();
    yield();
}
