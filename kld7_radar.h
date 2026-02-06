#include "esphome.h"

class KLD7Radar : public Component, public uart::UARTDevice {
 public:
  Sensor *distance_sensor = new Sensor();
  Sensor *speed_sensor    = new Sensor();
  BinarySensor *presence  = new BinarySensor();
  TextSensor *direction   = new TextSensor();

  KLD7Radar(UARTComponent *parent) : UARTDevice(parent) {}

  void setup() override {}

  void loop() override {
    static uint8_t buffer[32];
    static uint8_t idx = 0;
    static uint32_t last_seen = 0;

    while (available()) {
      uint8_t b;
      read_byte(&b);

      // Frame Sync 0xAA 0x55
      if (idx == 0 && b != 0xAA) continue;
      if (idx == 1 && b != 0x55) { idx = 0; continue; }

      buffer[idx++] = b;

      if (idx >= 10) {
        parse_frame_(buffer);
        idx = 0;
        last_seen = millis();
      }
    }

    if (presence->state && millis() - last_seen > 3000) {
      presence->publish_state(false);
      direction->publish_state("none");
    }
  }

 protected:
  void parse_frame_(uint8_t *f) {
    uint16_t dist_mm = f[4] | (f[5] << 8);
    int16_t speed_mm = f[6] | (f[7] << 8);

    float distance = dist_mm / 1000.0f;
    float speed    = speed_mm / 1000.0f;

    distance_sensor->publish_state(distance);
    speed_sensor->publish_state(speed);
    presence->publish_state(true);

    if (speed > 0.05f)
      direction->publish_state("approaching");
    else if (speed < -0.05f)
      direction->publish_state("leaving");
    else
      direction->publish_state("still");
  }
};
