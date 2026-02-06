#pragma once

#include <string>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "radarevent.h"

namespace esphome {
namespace kld7 {

const int DETECTION_TIMEOUT = 1500;
const float DETECTION_SPEED_DIFFERENCE = 20.0;
const uint16_t DETECTION_DISTANCE_DIFFERENCE = 1000;
const int PROCESS_MIN_POINTS = 5;
const int REQUEST_INTERVAL = 50;
const int TIMEOUT_INTERVAL = 500;

extern const char* PROCESSING_REASON_OBJECT_GONE;
extern const char* PROCESSING_REASON_TIME_OVERFLOW;
extern const char* PROCESSING_REASON_TIMEOUT;
extern const char* PROCESSING_REASON_DIRECTION_CHANGE;
extern const char* PROCESSING_REASON_SPEED_DIFFERENCE;
extern const char* PROCESSING_REASON_DISTANCE_DIFFERENCE;

class Sensor : public Component, public sensor::Sensor {};
class BinarySensor : public Component, public binary_sensor::BinarySensor {};
class TextSensor : public Component, public text_sensor::TextSensor {};

class Kld7 : public Component, public uart::UARTDevice {
 public:
  Kld7() {};
  void register_speed_sensor(sensor::Sensor* sensor) { this->_speed_sensor = sensor; };
  void register_avg_speed_sensor(sensor::Sensor* sensor) { this->_avg_speed_sensor = sensor; };
  void register_raw_speed_sensor(sensor::Sensor* sensor) { this->_raw_speed_sensor = sensor; };
  void register_points_sensor(sensor::Sensor* sensor) { this->_points_sensor = sensor; }
  void register_raw_angle_sensor(sensor::Sensor* sensor) { this->_raw_angle_sensor = sensor; }
  void register_raw_distance_sensor(sensor::Sensor* sensor) { this->_raw_distance_sensor = sensor; }
  void register_raw_magnitude_sensor(sensor::Sensor* sensor) { this->_raw_magnitude_sensor = sensor; }

  void register_raw_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_raw_detection_sensor = sensor; };
  void register_raw_direction_sensor(binary_sensor::BinarySensor* sensor) { this->_raw_direction_sensor = sensor; };
  void register_filtered_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_filtered_detection_sensor = sensor; };
  void register_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_detection_sensor = sensor; };


  void set_filtered_detection_sensor_min_distance(uint16_t value) { _filtered_sensor_min_distance = value; };
  void set_filtered_detection_sensor_max_distance(uint16_t value) { _filtered_sensor_max_distance = value; };
  void set_filtered_detection_sensor_min_angle(float value) { _filtered_sensor_min_angle = value; };
  void set_filtered_detection_sensor_max_angle(float value) { _filtered_sensor_max_angle = value; };
  void set_filtered_detection_sensor_min_points(uint16_t value) { _filtered_sensor_min_points = value; };
  void set_filtered_detection_sensor_timeout(uint16_t value) { _filtered_sensor_timeout = value; };

  void register_raw_json_sensor(text_sensor::TextSensor* sensor) { this->_raw_json_sensor = sensor; }
  void register_json_sensor(text_sensor::TextSensor* sensor) { this->_json_sensor = sensor; }

  void set_invert_angle(bool invert) { this->_invert_angle = invert; }

  void loop() override;
  void setup() override;
  void dump_config() override;
  bool _invert_angle = false;
  sensor::Sensor* _speed_sensor = NULL;
  sensor::Sensor* _avg_speed_sensor = NULL;
  sensor::Sensor* _raw_speed_sensor = NULL;
  sensor::Sensor* _points_sensor = NULL;
  sensor::Sensor* _raw_angle_sensor = NULL;
  sensor::Sensor* _raw_distance_sensor = NULL;
  sensor::Sensor* _raw_magnitude_sensor = NULL;
  uint16_t _filtered_sensor_min_distance = 0;
  uint16_t _filtered_sensor_max_distance = 3000;
  float _filtered_sensor_min_angle = -90.0;
  float _filtered_sensor_max_angle = 90.0;
  uint16_t _filtered_sensor_timeout = 1000;
  uint16_t _filtered_sensor_min_points = 5;

  binary_sensor::BinarySensor* _raw_detection_sensor = NULL;
  binary_sensor::BinarySensor* _detection_sensor = NULL;
  binary_sensor::BinarySensor* _raw_direction_sensor = NULL;
  binary_sensor::BinarySensor* _filtered_detection_sensor = NULL;

  text_sensor::TextSensor* _raw_json_sensor = NULL;
  text_sensor::TextSensor* _json_sensor = NULL;

  uint16_t _filtered_sensor_points = 0;
  unsigned long long _filtered_sensor_last_ts = 0;

 private:
  RawRadarEvent _last_raw;
  RawRadarEvent _previous_raw;
  ProcessedRadarEvent _current_process;
  bool _wait_for_ok();
  bool _waiting_for_data = false;
  void _process_detection();
  void _finish_processing(const char* reason);
  unsigned long long _last_request = 0;
  void _request_data();
};

}  // namespace kld7
}  // namespace esphome