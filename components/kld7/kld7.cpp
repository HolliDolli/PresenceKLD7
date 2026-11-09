#include "kld7.h"
#include "esphome/core/log.h"
namespace esphome {
namespace kld7 {
	
static const char* TAG = "KLD7";
unsigned long appstart = millis();
const char* PROCESSING_REASON_OBJECT_GONE = "object gone";
const char* PROCESSING_REASON_TIME_OVERFLOW = "time overflow";
const char* PROCESSING_REASON_TIMEOUT = "timeout";
const char* PROCESSING_REASON_DIRECTION_CHANGE = "direction changed";
const char* PROCESSING_REASON_SPEED_DIFFERENCE = "speed difference";
const char* PROCESSING_REASON_DISTANCE_DIFFERENCE = "distance difference";
void Kld7::setup() {
	
	ESP_LOGI(TAG, "Starting initialization of K-LD7...");
	ESP_LOGD(TAG, "Sending INIT");
	write_array((std::array<uint8_t, 12>){'I', 'N', 'I', 'T', 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending RSPI");
	write_array((std::array<uint8_t, 12>){'R', 'S', 'P', 'I', 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00}); // Maximum speed: 1  = 25km/h; 2 = 50km/h
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending RRAI");
	write_array((std::array<uint8_t, 12>){'R', 'R', 'A', 'I', 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00}); // Maximum range: 1 = 10m; 2 = 30m
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending THOF");
	write_array((std::array<uint8_t, 12>){'T', 'H', 'O', 'F', 0x04, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00}); // Threshold offset = 10dB
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending TRFT");
	write_array((std::array<uint8_t, 12>){'T', 'R', 'F', 'T', 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}); // Tracking filter type = Fast detection
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending MIRA"); // Minimum detection distance as % of range
	write_array((std::array<uint8_t, 12>){'M', 'I', 'R', 'A', 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // Minimum range = 0% der Range-Einstellung
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending MARA"); // Maximum detection distance as % of range
	write_array((std::array<uint8_t, 12>){'M', 'A', 'R', 'A', 0x04, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00}); // Maximum range = 100% der Range-Einstellung (RRAI=2 -> 30m)
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending MASP"); // Maximum detection speed as % of maxspeed
	write_array((std::array<uint8_t, 12>){'M', 'A', 'S', 'P', 0x04, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00}); // Maximum speed = 100% der Speed-Einstellung (RSPI=2 -> 50km/h)
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending RATH"); 
	write_array((std::array<uint8_t, 12>){'R', 'A', 'T', 'H', 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // Range threshold = 0%
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending SPTH");
	write_array((std::array<uint8_t, 12>){'S', 'P', 'T', 'H', 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // Speed threshold = 0%
	_wait_for_ok();
	ESP_LOGD(TAG, "Sending VISU");  // Vibration suppression default 2
	write_array((std::array<uint8_t, 12>){'V', 'I', 'S', 'U', 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00}); 
	_wait_for_ok();
	ESP_LOGI(TAG, "Initialization complete");
}
bool Kld7::_wait_for_ok() {
	unsigned long start = millis();
	while(true) {
		if (millis() - start > 500) {
			ESP_LOGE(TAG, "K-LD7 failed to respond within 500ms.");
//			mark_failed();
			return false;
		}
		if (available()>=9) {
			uint8_t data[9];
			read_array(data, 9);
			if (data[0]=='R' && data[1]=='E' && data[2]=='S' && data[3]=='P' &&
				data[4]==0x01 && data[5]==0x00&& data[6]==0x00 && data[7]==0x00 &&
				data[8]==0x00) return true;
			ESP_LOGE(TAG, "K-LD7 sent an unexpected answer during setup.");
			return false;
		}
	}
}
void Kld7::_request_data() {
	write_array((std::array<uint8_t, 12>){'G', 'N', 'F', 'D', 0x04, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00});
	_waiting_for_data = true;
	_last_request = millis();
}
void Kld7::loop() {
	if (_waiting_for_data == false && (_last_request > millis() || _last_request + REQUEST_INTERVAL < millis())) {
		//ESP_LOGD(TAG, "Sending GNFD");
		_request_data();
	} else if (_waiting_for_data && (_last_request > millis() || _last_request + TIMEOUT_INTERVAL < millis())) {
		ESP_LOGW(TAG, "Received no data. Sending another request.");
		_request_data();
	}
	// Header und Payload getrennt einlesen: Der Header wird gemerkt, bis die
	// komplette Payload im UART-Puffer liegt. So blockiert read_array() nie auf
	// noch nicht empfangene Bytes. Die Laenge aus dem Header wird begrenzt, damit
	// ein verstuemmelter Header kein riesiges Array auf dem Stack anlegt.
	// (Funktionslokale statics: es gibt nur eine Kld7-Instanz.)
	static const uint32_t MAX_PAYLOAD = 512;  // = 64 PDAT-Targets
	static bool have_head = false;
	static char command[5];
	static uint32_t length = 0;
	if (!have_head) {
		if (available() < 8) return;
		uint8_t head[8];
		read_array(head, 8);
		//ESP_LOGD(TAG, "Recieved data: %02X %02X %02X %02X %02X %02X %02X %02X", head[0], head[1], head[2], head[3], head[4], head[5], head[6], head[7]);
		memcpy(command, head, 4);
		command[4] = '\0';
		length = head[4] | (head[5]<<8) | (head[6]<<16) | (head[7]<<24);
		if (length > MAX_PAYLOAD) {
			ESP_LOGE(TAG, "Header %02X %02X %02X %02X mit unplausibler Laenge %lu - verwerfe UART-Puffer.", head[0], head[1], head[2], head[3], (unsigned long) length);
			while (available() > 0) read();
			return;
		}
		have_head = true;
	}
	if (available() < (int)length) return;
	have_head = false;
	{
		uint8_t payload[MAX_PAYLOAD];
		if (length > 0) read_array(payload, length);
		//ESP_LOGD(TAG, "Received command %s with a payload of %d bytes.", command, length);
		if (strcmp(command, "RESP")==0) {
			if (length != 1) {
				ESP_LOGE(TAG, "Received RESP with wrong payload length (expected: 1, was: %lu)", (unsigned long) length);
				return;
			}
			switch (payload[0]) {
				case 0: break; // OK
				case 1: ESP_LOGE(TAG, "RESP: Unknown command"); break;
				case 2: ESP_LOGE(TAG, "RESP: Invalid parameter value"); break;
				case 3: ESP_LOGE(TAG, "RESP: Invalid RPST version"); break;
				case 4: ESP_LOGE(TAG, "RESP: Uart error (parity, framing, noise)"); break;
				case 5: ESP_LOGE(TAG, "RESP: Sensor busy"); break;
				case 6: ESP_LOGE(TAG, "RESP: Timeout error"); break;
				default: ESP_LOGE(TAG, "RESP: Unknown error code %02X", payload[0]); break;
			}
		} else if(strcmp(command,"PDAT")==0	) {
			_process_pdat(payload, length);
		} else if(strcmp(command,"TDAT")==0) {
			if (length != 8 && length!=0) {
				ESP_LOGE(TAG, "Received TDAT with wrong payload length (expected: 0 or 8, was: %lu)", (unsigned long) length);
				return;
			}
			_last_raw = RawRadarEvent(payload, length, this->_invert_angle);
			if (_last_raw.detection) {
				//ESP_LOGD(TAG, "Raw data: %d cm, %.1f km/h, %.1f°, %.1fdB", _last_raw.distance, _last_raw.speed, _last_raw.angle, _last_raw.magnitude);
				if (_raw_speed_sensor != NULL) _raw_speed_sensor->publish_state(_last_raw.speed);
				if (_raw_angle_sensor != NULL) _raw_angle_sensor->publish_state(_last_raw.angle);
				if (_raw_distance_sensor != NULL) _raw_distance_sensor->publish_state(_last_raw.distance);
				if (_raw_magnitude_sensor != NULL) _raw_magnitude_sensor->publish_state(_last_raw.magnitude);
				if (_raw_direction_sensor != NULL) _raw_direction_sensor->publish_state(_last_raw.speed > 0);
				if (_raw_json_sensor != NULL) {
					char buff[64];
					snprintf(buff, sizeof(buff), "{\"speed\":%.1f,\"angle\":%.1f,\"distance\":%d,\"magnitude\":%.1f}", _last_raw.speed, _last_raw.angle, _last_raw.distance, _last_raw.magnitude);
					_raw_json_sensor->publish_state(buff);
				}
			 } else {
				//ESP_LOGD(TAG, "Raw data: No detection");
			 }
			 if (_raw_detection_sensor != NULL) _raw_detection_sensor->publish_state(_last_raw.detection);

			_process_filtered_detection();
			_process_detection();
			_waiting_for_data = false;
		} else {
			ESP_LOGW(TAG, "Received unexpected command %s. Ignoring.", command);
		}
	}	
}
void Kld7::_process_pdat(uint8_t* payload, uint32_t length) {
	int num_targets = length / 8;
	if (_num_targets_sensor != NULL && num_targets != _last_num_targets) {
		_num_targets_sensor->publish_state(num_targets);
		_last_num_targets = num_targets;
	}
	// Targets für die PDAT-Kohärenz-Validierung von filtered_detection merken.
	// Unabhängig davon, ob die JSON-Sensoren registriert sind - die Filterlogik
	// braucht die Rohdaten, nicht den JSON-String.
	_last_pdat_targets.clear();
	for (int i = 0; i < num_targets; i++) {
		_last_pdat_targets.push_back(RawRadarEvent(payload + i*8, 8, _invert_angle));
	}

	// Diagnose-JSON: kompakte 1-Buchstaben-Keys (d/s/a/m), aufgeteilt auf bis
	// zu 3 Sensoren à 5 Targets, weil ein einzelner TextSensor-State von HA
	// bei 255 Zeichen gekappt wird (auf "unknown" fällt) und PDAT-Frames in
	// der Praxis durchaus mehr als die 8 dokumentierten Targets liefern.
	// Targets 1-5 -> _pdat_json_sensor, 6-10 -> _pdat_json_sensor_2,
	// 11-15 -> _pdat_json_sensor_3. Alles darüber wird zur Sicherheit mit
	// "...]}" abgeschnitten statt ungültiges JSON zu erzeugen.
	const int TARGETS_PER_CHUNK = 5;
	text_sensor::TextSensor* chunk_sensors[3] = {_pdat_json_sensor, _pdat_json_sensor_2, _pdat_json_sensor_3};
	// Reihenfolge 3 -> 2 -> 1: Der HA-CSV-Logger triggert auf Chunk 1 und liest
	// dabei Chunk 2 und 3 mit. Die API uebertraegt der Reihe nach, also muessen
	// 2 und 3 vor 1 veroeffentlicht werden, sonst loggt HA dort den Vorframe.
	for (int chunk = 2; chunk >= 0; chunk--) {
		text_sensor::TextSensor* sens = chunk_sensors[chunk];
		if (sens == NULL) continue;
		int start = chunk * TARGETS_PER_CHUNK;
		if (start >= num_targets) {
			// Für diesen Chunk gibt es aktuell keine Targets - Sensor explizit
			// auf ein leeres, aber valides JSON-Objekt setzen, statt den
			// letzten (veralteten) Wert stehen zu lassen. "{}" statt "" ist
			// robuster für Templates/Automationen, die den State als JSON
			// parsen (json.loads("") würde sonst einen Fehler werfen).
			sens->publish_state("{}");
			continue;
		}
		int end = start + TARGETS_PER_CHUNK;
		if (end > num_targets) end = num_targets;

		char buff[260];
		int pos = snprintf(buff, sizeof(buff), "{\"n\":%d,\"t\":[", num_targets);
		bool truncated = false;
		for (int i = start; i < end; i++) {
			RawRadarEvent& target = _last_pdat_targets[i];
			// Sicherheitsmarge: bei worst-case Werten passt ein Target-Objekt
			// in ca. 45 Zeichen. Bricht die verbleibende Pufferkapazität das,
			// wird abgebrochen statt der Puffer zu überlaufen.
			if (pos > (int)sizeof(buff) - 50) { truncated = true; break; }
			pos += snprintf(buff+pos, sizeof(buff)-pos,
				"%s{\"d\":%d,\"s\":%.2f,\"a\":%.2f,\"m\":%.2f}",
				(i>start ? "," : ""), target.distance, target.speed, target.angle, target.magnitude);
		}
		snprintf(buff+pos, sizeof(buff)-pos, truncated ? "...]}" : "]}");
		sens->publish_state(buff);
		ESP_LOGD(TAG, "PDAT[%d]: %s", chunk+1, buff);
	}
}
void Kld7::_process_detection() {
	if (_last_raw.detection == false) {
		if (_current_process.active && (_current_process.timestamp + DETECTION_TIMEOUT < millis() || _current_process.timestamp > millis())) {
			// Object is gone. Stop processing.
			_finish_processing(PROCESSING_REASON_OBJECT_GONE);
		}
	} else {
		bool direction_away_from_radar = _last_raw.speed > 0;
		if (_current_process.timestamp > millis()) {
			_finish_processing(PROCESSING_REASON_TIME_OVERFLOW);
		} else if (millis() - _current_process.timestamp > DETECTION_TIMEOUT) {
			_finish_processing(PROCESSING_REASON_TIMEOUT);
		} else if (direction_away_from_radar != _current_process.direction_away_from_radar) {
			_finish_processing(PROCESSING_REASON_DIRECTION_CHANGE);
		} else if (fabs(_last_raw.speed - _current_process.last_speed)>DETECTION_SPEED_DIFFERENCE) {
			_finish_processing(PROCESSING_REASON_SPEED_DIFFERENCE);
		} else if (abs(_last_raw.distance - _current_process.last_distance)>DETECTION_DISTANCE_DIFFERENCE) {
			_finish_processing(PROCESSING_REASON_DISTANCE_DIFFERENCE);
		}
		// TODO save the data
		_current_process.active = true;
		_current_process.timestamp = millis();
		_current_process.points++;
		_current_process.direction_away_from_radar = direction_away_from_radar;
		_current_process.last_speed = _last_raw.speed;
		_current_process.last_distance = _last_raw.distance;
		float abs_speed = fabs(_last_raw.speed);
		_current_process.speed_sum += _last_raw.speed;
		_current_process.size_sum += _last_raw.distance * _last_raw.magnitude;
		if (abs_speed > fabs(_current_process.max_speed)) {
			_current_process.not_max_speed = _current_process.max_speed;
			_current_process.max_speed = _last_raw.speed;
		} else if (abs_speed > fabs(_current_process.not_max_speed)) {
			_current_process.not_max_speed = _last_raw.speed;
		}
	}
	_previous_raw = _last_raw;
}
void Kld7::_finish_processing(const char* reason) {
	_current_process.active = false;
	if (_current_process.points > PROCESS_MIN_POINTS) {
		float avg_speed = _current_process.speed_sum / _current_process.points;
		float avg_size = _current_process.size_sum / _current_process.points;
		if (_avg_speed_sensor != NULL) _avg_speed_sensor->publish_state(avg_speed);
		if (_avg_size_sensor != NULL) _avg_size_sensor->publish_state(avg_size);
		if (_points_sensor != NULL) _points_sensor->publish_state(_current_process.points);
		if (_speed_sensor != NULL) _speed_sensor->publish_state(_current_process.not_max_speed);
		if (_detection_sensor != NULL) _detection_sensor->publish_state(true);
		if (_json_sensor != NULL) {
			char buffer[100];
			snprintf(buffer, sizeof(buffer), "{\"speed\":%.1f,\"avg_speed\":%.1f,\"avg_size\":%.1f,\"points\":%d,\"reason\":\"%s\"}", _current_process.not_max_speed, avg_speed, avg_size, _current_process.points, reason);
			_json_sensor->publish_state(buffer);
		}
		ESP_LOGD(TAG, "_finish_processing (%s). %d points, maximum %f.1 km/h, average %f.1 km/h, direction_away_from_radar %d", reason, _current_process.points, _current_process.not_max_speed, avg_speed, _current_process.direction_away_from_radar ? 1 : 0);
	} else {
		ESP_LOGD(TAG, "_finish_processing (%s): Too little data. Ignoring event.", reason);
	}
	_current_process = ProcessedRadarEvent();
	if (_detection_sensor != NULL) _detection_sensor->publish_state(false);
}
bool Kld7::_pdat_validates(uint16_t reference_distance) {
	// Sucht unter den zuletzt empfangenen PDAT-Targets einen Punkt, der sowohl
	// nahe an reference_distance liegt (TDAT-Distanz desselben Zeitraums) als
	// auch selbst eine Mindestgeschwindigkeit aufweist. Das bestätigt, dass
	// die volle Punktwolke ein kohärent bewegtes Objekt zeigt und nicht nur
	// der reduzierte Tracking-Wert allein eine Bewegung suggeriert.
	for (auto& target : _last_pdat_targets) {
		if (fabs(target.speed) >= _filtered_pdat_speed_min) {
			int32_t diff = (int32_t)target.distance - (int32_t)reference_distance;
			if (abs(diff) <= (int32_t)_filtered_pdat_distance_tolerance) {
				return true;
			}
		}
	}
	return false;
}

void Kld7::_process_filtered_detection() {
	// Filterkriterien für "Eintreffen einer Person" (statt Winkel/Distanz-Fenster):
	//  1. Speed-Gate: |speed| >= _filtered_speed_min
	//  2. Persistenz-Gate: >= _filtered_min_run_frames aufeinanderfolgende
	//     Frames über dem Speed-Gate, mit max. _filtered_max_frame_gap ms
	//     Lücke zwischen zwei Frames.
	//  3. PDAT-Kohärenz-Gate: innerhalb von _filtered_validation_window ms
	//     nach Run-Start muss mindestens einmal ein passender, sich
	//     bewegender PDAT-Punkt gefunden werden (_pdat_validates()).
	// "direction changed" geht hier bewusst NICHT ein - siehe Begründung in
	// kld7.h bei den set_filtered_detection_*-Settern.
	if (_filtered_detection_sensor == NULL) return;

	unsigned long now = millis();

	if (_last_raw.detection && fabs(_last_raw.speed) >= _filtered_speed_min) {
		bool gap_too_large = _filtered_run_frames > 0 &&
			(_filtered_last_frame_ts > now || now - _filtered_last_frame_ts > _filtered_max_frame_gap);
		if (_filtered_run_frames == 0 || gap_too_large) {
			// Neuer Run beginnt.
			_filtered_run_frames = 0;
			_filtered_run_start_ts = now;
			_filtered_pdat_validations = 0;
		}
		_filtered_run_frames++;
		_filtered_last_frame_ts = now;

		// PDAT-Kohärenz nur innerhalb des Validierungsfensters ab Run-Start prüfen.
		// (_filtered_run_start_ts > now schützt gegen den millis()-Überlauf alle ~49 Tage,
		// analog zu den Checks in _process_detection()/_finish_processing().)
		bool within_window = _filtered_run_start_ts > now ||
			now - _filtered_run_start_ts <= _filtered_validation_window;
		if (within_window) {
			if (_pdat_validates(_last_raw.distance)) {
				_filtered_pdat_validations++;
			}
		}

		if (_filtered_run_frames >= _filtered_min_run_frames && _filtered_pdat_validations >= 1) {
			_filtered_detection_sensor->publish_state(true);
		}
	} else {
		// Frame unter Speed-Gate (oder keine Detection) -> Run beenden.
		// Bewusst kein "direction changed"-Check und keine Hysterese hier;
		// das Ende des Runs heißt nicht sofort "Person weg", siehe Timeout
		// unten für das Zurücksetzen von filtered_detection_sensor.
		_filtered_run_frames = 0;
		_filtered_pdat_validations = 0;
	}

	// Timeout-Hysterese: filtered_detection_sensor erst zurücksetzen, wenn
	// seit dem letzten validen Frame _filtered_sensor_timeout ms vergangen sind.
	// Verwendet weiterhin die bestehenden Parameter (set_filtered_detection_sensor_timeout),
	// damit kurze Lücken (z.B. Schritt-Rhythmus) nicht sofort zum Rücksetzen führen.
	if (_filtered_run_frames == 0) {
		if (_filtered_last_frame_ts > now || now - _filtered_last_frame_ts > _filtered_sensor_timeout) {
			_filtered_detection_sensor->publish_state(false);
		}
	}
}

void Kld7::dump_config() { 
	ESP_LOGCONFIG(TAG, "KLD7:"); 
}
} // namespace kld7
} // namespace esphome
