#include <WiFi.h>
#include <PubSubClient.h>

// ========= USER CONFIG =========
const char* WIFI_SSID     = "DEIN_WLAN";
const char* WIFI_PASSWORD = "DEIN_PASSWORT";

const char* MQTT_SERVER   = "192.168.1.10";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "";
const char* MQTT_PASSWORD = "";

// MQTT Topics
const char* TOPIC_PRESENCE = "radar/presence";
const char* TOPIC_RAW      = "radar/raw";

// UART Pins
#define RADAR_RX 16
#define RADAR_TX 17
// ===============================

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastRadarSeen = 0;
bool presenceState = false;

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect_mqtt() {
  while (!client.connected()) {
    if (client.connect("ESP32-RADAR", MQTT_USER, MQTT_PASSWORD)) {
      client.publish(TOPIC_PRESENCE, "false", true);
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Radar UART
  Serial2.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // Radar data available?
  while (Serial2.available()) {
    uint8_t buffer[64];
    int len = Serial2.readBytes(buffer, sizeof(buffer));

    // Timestamp merken
    lastRadarSeen = millis();

    // Presence setzen
    if (!presenceState) {
      presenceState = true;
      client.publish(TOPIC_PRESENCE, "true", true);
    }

    // Debug RAW als HEX senden
    char hexString[200] = {0};
    for (int i = 0; i < len && i < 32; i++) {
      sprintf(hexString + strlen(hexString), "%02X ", buffer[i]);
    }
    client.publish(TOPIC_RAW, hexString);
  }

  // Timeout → keine Präsenz
  if (presenceState && millis() - lastRadarSeen > 3000) {
    presenceState = false;
    client.publish(TOPIC_PRESENCE, "false", true);
  }
}
