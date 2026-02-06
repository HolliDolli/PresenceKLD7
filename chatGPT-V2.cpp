binary_sensor:
  - platform: mqtt
    name: "Radar Präsenz"
    state_topic: "radar/presence"
    payload_on: "true"
    payload_off: "false"
    device_class: occupancy

sensor:
  - platform: mqtt
    name: "Radar Entfernung"
    state_topic: "radar/distance"
    unit_of_measurement: "m"

  - platform: mqtt
    name: "Radar Geschwindigkeit"
    state_topic: "radar/speed"
    unit_of_measurement: "m/s"

  - platform: mqtt
    name: "Radar Richtung"
    state_topic: "radar/direction"




#include <WiFi.h>
#include <PubSubClient.h>

// ========= CONFIG =========
const char* WIFI_SSID = "DEIN_WLAN";
const char* WIFI_PASS = "DEIN_PASS";

const char* MQTT_HOST = "192.168.1.10";
const int   MQTT_PORT = 1883;

// UART
#define RADAR_RX 16
#define RADAR_TX 17

// MQTT Topics
#define T_PRESENCE  "radar/presence"
#define T_DISTANCE  "radar/distance"
#define T_DIRECTION "radar/direction"
#define T_SPEED     "radar/speed"
// ==========================

WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long lastSeen = 0;
bool presence = false;

// ---------- SETUP ----------
void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnectMQTT() {
  while (!mqtt.connected()) {
    mqtt.connect("ESP32-KLD7");
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  setupWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
}

// ---------- LOOP ----------
void loop() {
  if (!mqtt.connected()) reconnectMQTT();
  mqtt.loop();

  static uint8_t buffer[32];
  static int idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    // Frame Sync
    if (idx == 0 && b != 0xAA) continue;
    if (idx == 1 && b != 0x55) { idx = 0; continue; }

    buffer[idx++] = b;

    // Minimal Frame Length erreicht
    if (idx >= 10) {
      parseFrame(buffer);
      idx = 0;
    }
  }

  // Presence Timeout
  if (presence && millis() - lastSeen > 3000) {
    presence = false;
    mqtt.publish(T_PRESENCE, "false", true);
  }
}

// ---------- FRAME PARSER ----------
void parseFrame(uint8_t* f) {
  lastSeen = millis();

  if (!presence) {
    presence = true;
    mqtt.publish(T_PRESENCE, "true", true);
  }

  // Entfernung (mm → m)
  uint16_t dist_mm = f[4] | (f[5] << 8);
  float distance = dist_mm / 1000.0;

  // Geschwindigkeit (mm/s → m/s)
  int16_t speed_mm = f[6] | (f[7] << 8);
  float speed = speed_mm / 1000.0;

  // Richtung
  const char* direction = "still";
  if (speed > 0.05) direction = "approaching";
  else if (speed < -0.05) direction = "leaving";

  char buf[16];

  dtostrf(distance, 1, 2, buf);
  mqtt.publish(T_DISTANCE, buf, true);

  dtostrf(speed, 1, 2, buf);
  mqtt.publish(T_SPEED, buf, true);

  mqtt.publish(T_DIRECTION, direction, true);
}
