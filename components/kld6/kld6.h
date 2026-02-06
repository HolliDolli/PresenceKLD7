#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace kld6 {

class KLD6Component : public Component, public uart::UARTDevice {
 public:
  explicit KLD6Component(uart::UARTComponent *parent) : UARTDevice(parent) {}

  void set_distance_sensor(sensor::Sensor *s) { distance_ = s; }
  void set_speed_sensor(sensor::Sensor *s) { speed_ = s; }
  void set_direction_sensor(text_sensor::TextSensor *s) { direction_ = s; }
  void set_presence_sensor(binary_sensor::BinarySensor *s) { presence_ = s; }

  void set_presence_timeout(uint32_t ms) { presence_timeout_ = ms; }

  void loop() override;

 protected:
  void parse_frame_(uint8_t *frame);

  sensor::Sensor *distance_{nullptr};
  sensor::Sensor *speed_{nullptr};
  text_sensor::TextSensor *direction_{nullptr};
  binary_sensor::BinarySensor *presence_{nullptr};

  uint32_t last_seen_{0};
  uint32_t presence_timeout_{3000};
};

}  // namespace kld6
}  // namespace esphome
