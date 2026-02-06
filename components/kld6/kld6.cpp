#include "kld6.h"

namespace esphome {
namespace kld6 {

void KLD6Component::loop() {
  static uint8_t buffer[32];
  static uint8_t idx = 0;

  while (available()) {
    uint8_t b;
    read_byte(&b);

    if (idx == 0 && b != 0xAA) continue;
    if (idx == 1 && b != 0x55) { idx = 0; continue; }

    buffer[idx++] = b;

    if (idx >= 10) {
      parse_frame_(buffer);
      idx = 0;
      last_seen_ = millis();
    }
  }

  if (presence_ && presence_->state &&
      millis() - last_seen_ > presence_timeout_) {
    presence_->publish_state(false);
    if (direction_) direction_->publish_state("none");
  }
}

void KLD6Component::parse_frame_(uint8_t *f) {
  uint16_t dist_mm = f[4] | (f[5] << 8);
  int16_t speed_mm = f[6] | (f[7] << 8);

  float distance = dist_mm / 1000.0f;
  float speed = speed_mm / 1000.0f;

  if (distance_) distance_->publish_state(distance);
  if (speed_) speed_->publish_state(speed);

  if (presence_) presence_->publish_state(true);

  if (direction_) {
    if (speed > 0.05f)
      direction_->publish_state("approaching");
    else if (speed < -0.05f)
      direction_->publish_state("leaving");
    else
      direction_->publish_state("still");
  }
}

}  // namespace kld6
}  // namespace esphome
