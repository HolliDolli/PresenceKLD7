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
  void register_avg_size_sensor(sensor::Sensor* sensor) { this->_avg_size_sensor = sensor; };
  void register_raw_speed_sensor(sensor::Sensor* sensor) { this->_raw_speed_sensor = sensor; };
  void register_points_sensor(sensor::Sensor* sensor) { this->_points_sensor = sensor; }
  void register_raw_angle_sensor(sensor::Sensor* sensor) { this->_raw_angle_sensor = sensor; }
  void register_raw_distance_sensor(sensor::Sensor* sensor) { this->_raw_distance_sensor = sensor; }
  void register_raw_magnitude_sensor(sensor::Sensor* sensor) { this->_raw_magnitude_sensor = sensor; }
  void register_num_targets_sensor(sensor::Sensor* sensor) { this->_num_targets_sensor = sensor; }

  void register_raw_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_raw_detection_sensor = sensor; };
  void register_raw_direction_sensor(binary_sensor::BinarySensor* sensor) { this->_raw_direction_sensor = sensor; };
  void register_filtered_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_filtered_detection_sensor = sensor; };
  void register_detection_sensor(binary_sensor::BinarySensor* sensor) { this->_detection_sensor = sensor; };


  void set_filtered_detection_sensor_min_points(uint16_t value) { _filtered_sensor_min_points = value; };
  void set_filtered_detection_sensor_timeout(uint16_t value) { _filtered_sensor_timeout = value; };

  // Neue Filterkriterien: Speed-Persistenz + PDAT-Kohärenz statt Winkel/Distanz-Fenster.
  // Begründung siehe Analyse von KLD7-log-Kind-Erw.csv:
  //  - "direction changed" ist als Kriterium ungeeignet, da bei einem Gehweg orthogonal
  //    zum Sensor jede normale Querung am sensornächsten Punkt einen Vorzeichenwechsel
  //    der radialen Geschwindigkeit erzeugt (Geometrie, kein Personenverhalten).
  //  - magnitude trennt in den Testdaten NICHT zwischen Rauschen und Bewegung
  //    (Median ~29 in beiden Fällen) und wird daher bewusst nicht verwendet.
  void set_filtered_detection_speed_min(float value) { _filtered_speed_min = value; };
  void set_filtered_detection_min_run_frames(uint16_t value) { _filtered_min_run_frames = value; };
  void set_filtered_detection_max_frame_gap(uint16_t value) { _filtered_max_frame_gap = value; };
  void set_filtered_detection_pdat_distance_tolerance(uint16_t value) { _filtered_pdat_distance_tolerance = value; };
  void set_filtered_detection_pdat_speed_min(float value) { _filtered_pdat_speed_min = value; };
  void set_filtered_detection_validation_window(uint16_t value) { _filtered_validation_window = value; };

  void register_raw_json_sensor(text_sensor::TextSensor* sensor) { this->_raw_json_sensor = sensor; }
  void register_json_sensor(text_sensor::TextSensor* sensor) { this->_json_sensor = sensor; }
  void register_pdat_json_sensor(text_sensor::TextSensor* sensor) { this->_pdat_json_sensor = sensor; }
  // Bei mehr als 5 PDAT-Targets reicht ein 255-Zeichen-TextSensor nicht mehr
  // aus (HA kappt den State sonst auf "unknown"). Die Targets 6-10 und 11-15
  // gehen in zwei weitere Sensoren, je 5 Targets, kompakte 1-Buchstaben-Keys.
  void register_pdat_json_sensor_2(text_sensor::TextSensor* sensor) { this->_pdat_json_sensor_2 = sensor; }
  void register_pdat_json_sensor_3(text_sensor::TextSensor* sensor) { this->_pdat_json_sensor_3 = sensor; }
  void set_invert_angle(bool invert) { this->_invert_angle = invert; }

  void loop() override;
  void setup() override;
  void dump_config() override;
  bool _invert_angle = false;
  
  
  
  sensor::Sensor* _speed_sensor = NULL;
  sensor::Sensor* _avg_speed_sensor = NULL;
  sensor::Sensor* _avg_size_sensor = NULL;
  sensor::Sensor* _raw_speed_sensor = NULL;
  sensor::Sensor* _points_sensor = NULL;
  sensor::Sensor* _raw_angle_sensor = NULL;
  sensor::Sensor* _raw_distance_sensor = NULL;
  sensor::Sensor* _raw_magnitude_sensor = NULL;
  sensor::Sensor* _num_targets_sensor = NULL;
  uint16_t _filtered_sensor_timeout = 1000;
  uint16_t _filtered_sensor_min_points = 5;

  // --- Neuer Filterzustand: Speed-Persistenz ---
  // Schwellenwert für |speed|, ab dem ein TDAT-Frame als "Bewegung" zählt.
  float _filtered_speed_min = 0.8;
  // Mindestanzahl aufeinanderfolgender Frames über _filtered_speed_min, bevor
  // überhaupt eine Detection in Erwägung gezogen wird.
  uint16_t _filtered_min_run_frames = 4;
  // Maximale Lücke (ms) zwischen zwei Frames, die noch als "derselbe Run" gilt.
  uint16_t _filtered_max_frame_gap = 500;
  // --- Neuer Filterzustand: PDAT-Kohärenz-Validierung ---
  // Toleranz (cm) zwischen TDAT-Distanz und einem validierenden PDAT-Punkt.
  uint16_t _filtered_pdat_distance_tolerance = 80;
  // Mindestgeschwindigkeit eines PDAT-Punktes, damit er als Validierung zählt.
  float _filtered_pdat_speed_min = 0.8;
  // Zeitfenster (ms) ab Run-Start, in dem eine PDAT-Validierung noch zählt.
  uint16_t _filtered_validation_window = 1200;

  binary_sensor::BinarySensor* _raw_detection_sensor = NULL;
  binary_sensor::BinarySensor* _detection_sensor = NULL;
  binary_sensor::BinarySensor* _raw_direction_sensor = NULL;
  binary_sensor::BinarySensor* _filtered_detection_sensor = NULL;

  text_sensor::TextSensor* _raw_json_sensor = NULL;
  text_sensor::TextSensor* _json_sensor = NULL;
  text_sensor::TextSensor* _pdat_json_sensor = NULL;
  text_sensor::TextSensor* _pdat_json_sensor_2 = NULL;
  text_sensor::TextSensor* _pdat_json_sensor_3 = NULL;

  // Laufzeit-Zustand des aktuellen Bewegungs-Runs (Speed-Persistenz).
  uint16_t _filtered_run_frames = 0;
  unsigned long _filtered_run_start_ts = 0;
  unsigned long _filtered_last_frame_ts = 0;
  // Anzahl der PDAT-Validierungen, die für den aktuellen Run bereits eingetroffen sind.
  uint16_t _filtered_pdat_validations = 0;

 private:
  RawRadarEvent _last_raw;
  RawRadarEvent _previous_raw;
  ProcessedRadarEvent _current_process;
  // Targets aus dem zuletzt empfangenen PDAT-Frame, für die PDAT-Kohärenz-
  // Validierung der filtered_detection. Wird in _process_pdat() befüllt.
  std::vector<RawRadarEvent> _last_pdat_targets;
  bool _wait_for_ok();
  bool _waiting_for_data = false;
  void _process_detection();
  void _finish_processing(const char* reason);
  void _process_pdat(uint8_t* payload, uint32_t length);
  // Prüft, ob unter den zuletzt empfangenen PDAT-Targets ein zur TDAT-Distanz
  // passender, sich bewegender Punkt vorhanden ist (PDAT-Kohärenz-Gate).
  bool _pdat_validates(uint16_t reference_distance);
  // Wertet TDAT-Frame gegen Speed-Persistenz + PDAT-Kohärenz aus und
  // publiziert ggf. filtered_detection_sensor.
  void _process_filtered_detection();
  int _last_num_targets = -1;  // -1 sorgt dafür, dass der erste echte Wert (auch 0) immer published wird  
  unsigned long long _last_request = 0;
  void _request_data();
};

}  // namespace kld7
}  // namespace esphome