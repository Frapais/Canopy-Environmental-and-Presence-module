/*
 * Canopy + Sprig-C3 — MQTT environmental node
 * ------------------------------------------------------------------
 * Hardware:
 *   Sprig-C3 (ESP32-C3) + Canopy expansion PCB
 *   I2C bus on SDA=GPIO2, SCL=GPIO3
 *     0x36  MAX17048  fuel gauge   (on the Sprig-C3 itself)
 *     0x40  HDC1080   temp / RH    (power via GPIO8)
 *     0x53  ENS160    eCO2 / TVOC  (power via GPIO0)
 *     0x48  VEML6030  ambient lux  (power via GPIO7)
 *
 * Behaviour:
 *   First boot (or after factory reset) -> opens the "Canopy-Setup" AP with a
 *   captive portal at 192.168.4.1 where the user enters WiFi + MQTT details.
 *   Credentials are stored in NVS; on later boots the node connects, publishes
 *   Home Assistant MQTT discovery, then publishes all readings every 60 s.
 *
 *   Hold BOOT (GPIO9) for 5 s at any time to wipe credentials and reopen
 *   the setup portal.
 *
 * Required libraries (Library Manager names):
 *   PubSubClient                        by Nick O'Leary
 *   ArduinoJson  (v6.x)                 by Benoit Blanchon
 *   Adafruit MAX1704X                   by Adafruit
 *   ClosedCube HDC1080                  by ClosedCube
 *   SparkFun Indoor Air Quality Sensor - ENS160
 *   SparkFun Ambient Light Sensor Arduino Library (VEML6030)
 *
 * Board: "ESP32C3 Dev Module" (esp32 core 2.0.x / 3.x)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "Adafruit_MAX1704X.h"
#include "ClosedCube_HDC1080.h"
#include "SparkFun_ENS160.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

// ------------------------------------------------------------------
//  User-tunable settings
// ------------------------------------------------------------------
#define SDA_PIN            2
#define SCL_PIN            3

#define PWR_HDC1080        8      // GPIO powering the HDC1080
#define PWR_ENS160         0      // GPIO powering the ENS160
#define PWR_VEML6030       7      // GPIO powering the VEML6030

#define BOOT_BTN           9      // Sprig-C3 BOOT button, active low
#define BOOT_HOLD_MS       5000   // hold time for factory reset

#define PUBLISH_INTERVAL   60000UL   // ms between publishes
#define WIFI_TIMEOUT_MS    20000UL   // give up and open the portal after this

#define VEML6030_ADDR      0x48   // 0x48 if ADDR is high, 0x10 if ADDR is low
#define ENS160_ADDR        0x53   // 0x53 if ADDR is high, 0x52 if ADDR is low

// The ENS160 needs ~3 min of warm-up after every power-up before its gas
// readings mean anything, so with a 60 s interval it is left powered on.
// Set this to false only if you switch to a long deep-sleep cycle.
#define KEEP_SENSORS_POWERED   true

const char *AP_SSID = "Canopy-Setup";
const char *AP_PASS = "";          // open AP; set an 8+ char password to secure it
const char *FW_VERSION = "1.0";

// ------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------
Adafruit_MAX17048   maxlipo;
ClosedCube_HDC1080  hdc1080;
SparkFun_ENS160     ens160;
SparkFun_Ambient_Light veml(VEML6030_ADDR);

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WebServer    server(80);
DNSServer    dnsServer;
Preferences  prefs;

// stored configuration
String cfgSsid, cfgPass, cfgMqttHost, cfgMqttUser, cfgMqttPass;
int    cfgMqttPort = 1883;
String cfgDeviceName = "Canopy";

// runtime state
bool  portalMode      = false;
bool  discoverySent   = false;
bool  hdcOk = false, ensOk = false, vemlOk = false, fuelOk = false;
char  devUniqueID[32];
String baseTopic, stateTopic, availTopic, deviceIP;
unsigned long lastPublish = 0;
unsigned long bootPressStart = 0;
float vemlGain = 0.125;     // low gain suits indoor + daylight without saturating
int   vemlIntegTime = 100;  // ms

// ------------------------------------------------------------------
//  Sensor power control
// ------------------------------------------------------------------
void sensorsPower(bool on) {
  digitalWrite(PWR_HDC1080,  on ? HIGH : LOW);
  digitalWrite(PWR_ENS160,   on ? HIGH : LOW);
  digitalWrite(PWR_VEML6030, on ? HIGH : LOW);
  if (on) delay(50);   // let the rails settle before touching I2C
}

// ------------------------------------------------------------------
//  Sensor init
// ------------------------------------------------------------------
void initSensors() {
  // --- MAX17048 fuel gauge (always powered, lives on the Sprig-C3) ---
  fuelOk = maxlipo.begin(&Wire);
  if (fuelOk) {
    Serial.print(F("MAX17048 chip ID: 0x"));
    Serial.println(maxlipo.getChipID(), HEX);
  } else {
    Serial.println(F("MAX17048 not found."));
  }

  // --- HDC1080 ---
  hdc1080.begin(0x40);
  delay(20);
  float t = hdc1080.readTemperature();
  hdcOk = !isnan(t) && t > -40.0 && t < 125.0;
  Serial.println(hdcOk ? F("HDC1080 ready.") : F("HDC1080 not responding."));

  // --- VEML6030 ---
  vemlOk = veml.begin();
  if (vemlOk) {
    veml.setGain(vemlGain);
    veml.setIntegTime(vemlIntegTime);
    Serial.println(F("VEML6030 ready."));
  } else {
    Serial.println(F("VEML6030 not responding."));
  }

  // --- ENS160 ---
  ensOk = ens160.begin(ENS160_ADDR);
  if (ensOk) {
    ens160.setOperatingMode(SFE_ENS160_RESET);
    delay(100);
    ens160.setOperatingMode(SFE_ENS160_STANDARD);
    delay(100);
    Serial.println(F("ENS160 ready (3 min warm-up before valid gas data)."));
  } else {
    Serial.println(F("ENS160 not responding."));
  }
}

void i2cScan() {
  Serial.println(F("I2C scan:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found 0x"));
      Serial.println(a, HEX);
    }
  }
}

// ------------------------------------------------------------------
//  Unique ID + topics
// ------------------------------------------------------------------
void buildIdentity() {
  byte mac[6];
  WiFi.macAddress(mac);
  snprintf(devUniqueID, sizeof(devUniqueID), "canopy%02X%02X%02X",
           mac[3], mac[4], mac[5]);

  baseTopic  = String("canopy/") + devUniqueID;
  stateTopic = baseTopic + "/state";
  availTopic = baseTopic + "/status";

  Serial.print(F("Device ID: "));
  Serial.println(devUniqueID);
}

// ------------------------------------------------------------------
//  Configuration portal
// ------------------------------------------------------------------
String portalPage() {
  String p = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
               "<meta name='viewport' content='width=device-width,initial-scale=1'>"
               "<title>Canopy Setup</title><style>"
               "body{font-family:system-ui,sans-serif;margin:0;padding:24px;"
               "background:#12161a;color:#e8eef2}"
               ".c{max-width:420px;margin:0 auto}"
               "h1{font-size:20px;margin:0 0 4px}p.s{color:#8fa3b0;margin:0 0 20px;font-size:14px}"
               "label{display:block;margin:14px 0 4px;font-size:13px;color:#a8bcc8}"
               "input{width:100%;box-sizing:border-box;padding:10px;border-radius:6px;"
               "border:1px solid #2b343c;background:#1a2027;color:#e8eef2;font-size:15px}"
               "button{width:100%;margin-top:22px;padding:12px;border:0;border-radius:6px;"
               "background:#3d9970;color:#fff;font-size:16px;font-weight:600}"
               "</style></head><body><div class='c'>"
               "<h1>Canopy Setup</h1><p class='s'>WiFi and MQTT broker details</p>"
               "<form method='POST' action='/save'>");
  p += F("<label>Device name</label><input name='dev_name' value='Canopy' maxlength='24'>");
  p += F("<label>WiFi SSID</label><input name='wifi_ssid' maxlength='32' required>");
  p += F("<label>WiFi password</label><input name='wifi_pass' type='password' maxlength='64'>");
  p += F("<label>MQTT broker (IP or host)</label><input name='mqtt_host' required>");
  p += F("<label>MQTT port</label><input name='mqtt_port' value='1883'>");
  p += F("<label>MQTT username</label><input name='mqtt_user'>");
  p += F("<label>MQTT password</label><input name='mqtt_pass' type='password'>");
  p += F("<button type='submit'>Save &amp; reboot</button></form></div></body></html>");
  return p;
}

void startConfigPortal() {
  Serial.println(F("Starting configuration portal."));
  portalMode = true;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, strlen(AP_PASS) ? AP_PASS : NULL);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print(F("Connect to '"));
  Serial.print(AP_SSID);
  Serial.print(F("' then browse to http://"));
  Serial.println(apIP);

  dnsServer.start(53, "*", apIP);   // captive portal redirect

  server.on("/", HTTP_GET, []() { server.send(200, "text/html", portalPage()); });

  server.on("/save", HTTP_POST, []() {
    prefs.begin("canopy", false);
    prefs.putString("wifi_ssid", server.arg("wifi_ssid"));
    prefs.putString("wifi_pass", server.arg("wifi_pass"));
    prefs.putString("mqtt_host", server.arg("mqtt_host"));
    prefs.putInt   ("mqtt_port", server.arg("mqtt_port").toInt() ? server.arg("mqtt_port").toInt() : 1883);
    prefs.putString("mqtt_user", server.arg("mqtt_user"));
    prefs.putString("mqtt_pass", server.arg("mqtt_pass"));
    prefs.putString("dev_name",  server.arg("dev_name").length() ? server.arg("dev_name") : String("Canopy"));
    prefs.end();
    server.send(200, "text/html",
      F("<html><body style='font-family:sans-serif;background:#12161a;color:#e8eef2;padding:40px'>"
        "<h3>Saved. Rebooting…</h3></body></html>"));
    delay(600);
    ESP.restart();
  });

  // anything else -> portal page, so phones pop the captive portal sheet
  server.onNotFound([]() { server.send(200, "text/html", portalPage()); });

  server.begin();
}

void startRuntimeServer() {
  server.on("/", HTTP_GET, []() {
    String p = F("<html><head><meta charset='utf-8'><meta name='viewport' "
                 "content='width=device-width,initial-scale=1'><title>Canopy</title>"
                 "<style>body{font-family:system-ui,sans-serif;background:#12161a;"
                 "color:#e8eef2;padding:24px}code{color:#7fd1a8}</style></head><body>");
    p += "<h2>" + cfgDeviceName + "</h2>";
    p += "<p>ID: <code>" + String(devUniqueID) + "</code></p>";
    p += "<p>Broker: <code>" + cfgMqttHost + ":" + String(cfgMqttPort) + "</code> — " +
         (mqtt.connected() ? "connected" : "disconnected") + "</p>";
    p += "<p>State topic: <code>" + stateTopic + "</code></p>";
    p += "<p>RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
    p += "<p>Sensors: HDC1080 " + String(hdcOk ? "ok" : "fail") +
         " · ENS160 " + String(ensOk ? "ok" : "fail") +
         " · VEML6030 " + String(vemlOk ? "ok" : "fail") + "</p>";
    p += F("<p><a href='/reset' style='color:#e08a8a'>Erase credentials &amp; reboot</a></p>");
    p += F("</body></html>");
    server.send(200, "text/html", p);
  });

  server.on("/reset", HTTP_GET, []() {
    server.send(200, "text/html", F("<html><body>Erasing…</body></html>"));
    prefs.begin("canopy", false);
    prefs.clear();
    prefs.end();
    delay(400);
    ESP.restart();
  });

  server.begin();
}

// ------------------------------------------------------------------
//  Home Assistant MQTT discovery
// ------------------------------------------------------------------
void addDeviceBlock(JsonDocument &doc) {
  JsonObject dev = doc.createNestedObject("dev");
  JsonArray ids = dev.createNestedArray("ids");
  ids.add(devUniqueID);
  dev["name"] = cfgDeviceName;
  dev["mf"]   = "Sprig Labs";
  dev["mdl"]  = "Sprig-C3 + Canopy";
  dev["sw"]   = FW_VERSION;
  dev["cu"]   = String("http://") + deviceIP + "/";
}

void publishDiscovery(const char *key, const char *name, const char *unit,
                      const char *devClass, const char *stateClass,
                      const char *icon = nullptr, bool diagnostic = false) {
  StaticJsonDocument<640> doc;
  char uid[64], topic[160];

  snprintf(uid, sizeof(uid), "%s_%s", devUniqueID, key);
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/config", uid);

  doc["name"]     = name;
  doc["uniq_id"]  = uid;
  doc["obj_id"]   = uid;
  doc["stat_t"]   = stateTopic;
  doc["avty_t"]   = availTopic;
  doc["val_tpl"]  = String("{{ value_json.") + key + " }}";
  if (unit)       doc["unit_of_meas"] = unit;
  if (devClass)   doc["dev_cla"]      = devClass;
  if (stateClass) doc["stat_cla"]     = stateClass;
  if (icon)       doc["ic"]           = icon;
  if (diagnostic) doc["ent_cat"]      = "diagnostic";
  doc["exp_aft"] = (PUBLISH_INTERVAL / 1000) * 3;   // mark unavailable after 3 missed cycles
  addDeviceBlock(doc);

  char payload[640];
  size_t n = serializeJson(doc, payload, sizeof(payload));
  if (!mqtt.publish(topic, (const uint8_t *)payload, n, true)) {
    Serial.print(F("Discovery publish failed for "));
    Serial.println(key);
  }
}

void sendAllDiscovery() {
  publishDiscovery("temperature", "Temperature",  "°C",  "temperature",  "measurement");
  publishDiscovery("humidity",    "Humidity",     "%",   "humidity",     "measurement");
  publishDiscovery("eco2",        "eCO2",         "ppm", "carbon_dioxide", "measurement");
  publishDiscovery("tvoc",        "TVOC",         "ppb", "volatile_organic_compounds_parts", "measurement");
  publishDiscovery("aqi",         "Air Quality Index", nullptr, "aqi",  "measurement");
  publishDiscovery("illuminance", "Illuminance",  "lx",  "illuminance",  "measurement");
  publishDiscovery("bat_voltage", "Battery Voltage", "V", "voltage",     "measurement", nullptr, true);
  publishDiscovery("bat_level",   "Battery",      "%",   "battery",      "measurement", nullptr, true);
  publishDiscovery("rssi",        "WiFi Signal",  "dBm", "signal_strength", "measurement", nullptr, true);
  discoverySent = true;
  Serial.println(F("Discovery published."));
}

// ------------------------------------------------------------------
//  Reading + publishing
// ------------------------------------------------------------------
void readAndPublish() {
  StaticJsonDocument<384> doc;

  float tempC = NAN, rh = NAN;

  if (hdcOk) {
    tempC = hdc1080.readTemperature();
    rh    = hdc1080.readHumidity();
    if (!isnan(tempC) && tempC > -40 && tempC < 125) {
      doc["temperature"] = serialized(String(tempC, 2));
      doc["humidity"]    = serialized(String(rh, 1));
    }
  }

  if (ensOk) {
    // feed the HDC1080 readings in as compensation before sampling
    if (!isnan(tempC)) {
      ens160.setTempCompensationCelsius(tempC);
      ens160.setRHCompensationFloat(rh);
    }
    if (ens160.checkDataStatus()) {
      doc["eco2"] = ens160.getECO2();
      doc["tvoc"] = ens160.getTVOC();
      doc["aqi"]  = ens160.getAQI();
    } else {
      Serial.println(F("ENS160 data not ready (still warming up?)."));
    }
  }

  if (vemlOk) {
    float lux = veml.readLight();
    if (lux >= 0) doc["illuminance"] = serialized(String(lux, 1));
  }

  if (fuelOk) {
    doc["bat_voltage"] = serialized(String(maxlipo.cellVoltage(), 3));
    doc["bat_level"]   = serialized(String(maxlipo.cellPercent(), 1));
  }

  doc["rssi"] = WiFi.RSSI();

  char payload[384];
  size_t n = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(stateTopic.c_str(), (const uint8_t *)payload, n, true);

  Serial.print(F("Published: "));
  Serial.println(payload);
}

// ------------------------------------------------------------------
//  WiFi / MQTT
// ------------------------------------------------------------------
bool connectWiFi(uint32_t timeoutMs) {
  Serial.print(F("Connecting to "));
  Serial.println(cfgSsid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);            // modem sleep between beacons saves ~20 mA
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi connection failed."));
    return false;
  }
  deviceIP = WiFi.localIP().toString();
  Serial.print(F("Connected, IP "));
  Serial.println(deviceIP);
  return true;
}

bool mqttConnect() {
  Serial.print(F("Connecting to MQTT broker… "));
  bool ok;
  if (cfgMqttUser.length()) {
    ok = mqtt.connect(devUniqueID, cfgMqttUser.c_str(), cfgMqttPass.c_str(),
                      availTopic.c_str(), 0, true, "offline");
  } else {
    ok = mqtt.connect(devUniqueID, availTopic.c_str(), 0, true, "offline");
  }

  if (ok) {
    Serial.println(F("connected."));
    mqtt.publish(availTopic.c_str(), "online", true);
    sendAllDiscovery();
    return true;
  }
  Serial.print(F("failed, rc="));
  Serial.println(mqtt.state());
  return false;
}

// ------------------------------------------------------------------
//  Setup
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\nCanopy / Sprig-C3 starting…"));

  pinMode(BOOT_BTN, INPUT_PULLUP);

  pinMode(PWR_HDC1080,  OUTPUT);
  pinMode(PWR_ENS160,   OUTPUT);
  pinMode(PWR_VEML6030, OUTPUT);
  sensorsPower(true);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  i2cScan();
  initSensors();

  // ---- load stored config ----
  prefs.begin("canopy", true);
  cfgSsid       = prefs.getString("wifi_ssid", "");
  cfgPass       = prefs.getString("wifi_pass", "");
  cfgMqttHost   = prefs.getString("mqtt_host", "");
  cfgMqttUser   = prefs.getString("mqtt_user", "");
  cfgMqttPass   = prefs.getString("mqtt_pass", "");
  cfgMqttPort   = prefs.getInt   ("mqtt_port", 1883);
  cfgDeviceName = prefs.getString("dev_name", "Canopy");
  prefs.end();

  buildIdentity();

  // ---- BOOT held at power-up -> wipe and open portal ----
  if (digitalRead(BOOT_BTN) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(BOOT_BTN) == LOW && millis() - t0 < BOOT_HOLD_MS) delay(10);
    if (digitalRead(BOOT_BTN) == LOW) {
      Serial.println(F("BOOT held — clearing stored credentials."));
      prefs.begin("canopy", false);
      prefs.clear();
      prefs.end();
      cfgSsid = "";
    }
  }

  // ---- no credentials, or WiFi unreachable -> portal ----
  if (cfgSsid.length() == 0 || cfgMqttHost.length() == 0) {
    startConfigPortal();
    return;
  }
  if (!connectWiFi(WIFI_TIMEOUT_MS)) {
    startConfigPortal();
    return;
  }

  mqtt.setBufferSize(1024);        // discovery payloads exceed the 256 B default
  mqtt.setServer(cfgMqttHost.c_str(), cfgMqttPort);
  mqtt.setKeepAlive(90);
  mqttConnect();

  startRuntimeServer();
  lastPublish = millis() - PUBLISH_INTERVAL;   // publish immediately on first loop
}

// ------------------------------------------------------------------
//  Loop
// ------------------------------------------------------------------
void loop() {
  // factory-reset long press, works in both modes
  if (digitalRead(BOOT_BTN) == LOW) {
    if (bootPressStart == 0) bootPressStart = millis();
    else if (millis() - bootPressStart >= BOOT_HOLD_MS) {
      Serial.println(F("Factory reset."));
      prefs.begin("canopy", false);
      prefs.clear();
      prefs.end();
      delay(200);
      ESP.restart();
    }
  } else {
    bootPressStart = 0;
  }

  if (portalMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(2);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi lost, reconnecting…"));
    WiFi.reconnect();
    delay(2000);
    return;
  }

  if (!mqtt.connected()) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {
      lastAttempt = millis();
      mqttConnect();
    }
  } else {
    mqtt.loop();
  }

  server.handleClient();

  if (mqtt.connected() && millis() - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = millis();
    readAndPublish();
  }
}
