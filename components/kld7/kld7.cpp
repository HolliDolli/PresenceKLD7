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
// Initialisierungsfolge fuer den K-LD7. Jeder Eintrag ist ein Kommando mit
// 4 Byte Payload (Wert im ersten Byte).
static const struct { const char* name; uint8_t value; } INIT_SEQUENCE[] = {
	{"INIT", 0x00},  // Verbindung aufbauen, 115200 Baud
	{"RSPI", 0x02},  // Maximum speed: 1 = 25km/h; 2 = 50km/h
	{"RRAI", 0x02},  // Maximum range: 1 = 10m; 2 = 30m
	{"THOF", 0x0a},  // Threshold offset = 10dB
	{"TRFT", 0x01},  // Tracking filter type = Fast detection
	{"MIRA", 0x00},  // Minimum detection distance = 0% der Range-Einstellung
	{"MARA", 0x64},  // Maximum detection distance = 100% der Range-Einstellung (RRAI=2 -> 30m)
	{"MASP", 0x64},  // Maximum detection speed = 100% der Speed-Einstellung (RSPI=2 -> 50km/h)
	{"RATH", 0x00},  // Range threshold = 0%
	{"SPTH", 0x00},  // Speed threshold = 0%
	{"VISU", 0x02},  // Vibration suppression, default 2
};

void Kld7::setup() {
	ESP_LOGI(TAG, "Starting initialization of K-LD7...");
	if (_init_sensor()) {
		ESP_LOGI(TAG, "Initialization complete");
	} else {
		ESP_LOGE(TAG, "Initialization failed - wird aus loop() wiederholt.");
	}
}
bool Kld7::_init_sensor() {
	// Wird von setup() und - nach REINIT_AFTER_TIMEOUTS Timeouts in Folge - von
	// loop() aufgerufen. Bricht beim ersten nicht quittierten Kommando ab, damit
	// ein nicht antwortender Sensor loop() hoechstens 500 ms blockiert. Ein
	// halb initialisierter Sensor (Werkseinstellungen!) bleibt so nicht
	// unbemerkt: der naechste Versuch folgt automatisch.
	_have_head = false;
	_waiting_for_data = false;
	_pdat_pending = false;
	_tdat_pending = false;
	_have_last_frame_no = false;  // der Sensor zaehlt nach einem Neustart wieder ab 0
	for (const auto& cmd : INIT_SEQUENCE) {
		ESP_LOGD(TAG, "Sending %s", cmd.name);
		while (available() > 0) read();  // Reste verwerfen, sonst wird die Quittung falsch gelesen
		write_array((std::array<uint8_t, 12>){(uint8_t)cmd.name[0], (uint8_t)cmd.name[1], (uint8_t)cmd.name[2], (uint8_t)cmd.name[3],
			0x04, 0x00, 0x00, 0x00, cmd.value, 0x00, 0x00, 0x00});
		if (!_wait_for_ok()) {
			ESP_LOGE(TAG, "%s wurde nicht quittiert - Initialisierung abgebrochen.", cmd.name);
			return false;
		}
	}
	return true;
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
	// GNFD-Bitmaske: 0x04 = PDAT, 0x08 = TDAT, 0x20 = DONE (Frame-Nummer des Sensors).
	uint8_t mask = _use_done ? 0x2C : 0x0C;
	write_array((std::array<uint8_t, 12>){'G', 'N', 'F', 'D', 0x04, 0x00, 0x00, 0x00, mask, 0x00, 0x00, 0x00});
	_waiting_for_data = true;
	_last_request = millis();
}
void Kld7::loop() {
	if (_waiting_for_data == false && (_last_request > millis() || _last_request + REQUEST_INTERVAL < millis())) {
		// Nur beim Start bzw. nach einer Re-Initialisierung. Im laufenden Betrieb
		// fordert _finish_frame() den naechsten Frame sofort an.
		_request_data();
	} else if (_waiting_for_data && (_last_request > millis() || _last_request + TIMEOUT_INTERVAL < millis())) {
		ESP_LOGW(TAG, "Received no data. Sending another request.");
		if (_use_done && _tdat_pending && ++_done_missing >= 3) {
			// TDAT kam an, DONE aber dreimal in Folge nicht: Dieser Sensor liefert
			// offenbar kein DONE. Ohne Sensor-Frame-Nummer weiterarbeiten.
			ESP_LOGE(TAG, "K-LD7 sendet kein DONE - Frame-Nummer wird ab jetzt vom ESP gezaehlt, verpasste Frames sind dann nicht erkennbar.");
			_use_done = false;
		}
		_pdat_pending = false;  // unvollstaendigen Frame verwerfen
		_tdat_pending = false;
		if (++_timeout_count >= REINIT_AFTER_TIMEOUTS) {
			// Der Sensor antwortet dauerhaft nicht (z.B. nach eigenem Neustart
			// ohne Verbindung, oder setup() ist fehlgeschlagen): neu verbinden
			// und alle Parameter erneut setzen.
			ESP_LOGE(TAG, "%d Timeouts in Folge - initialisiere K-LD7 neu.", REINIT_AFTER_TIMEOUTS);
			_timeout_count = 0;
			if (_init_sensor()) ESP_LOGI(TAG, "Re-Initialisierung erfolgreich.");
		}
		_request_data();
	}
	// Alle vollstaendig empfangenen Pakete in EINEM Durchlauf verarbeiten
	// (RESP, PDAT, TDAT, DONE). Frueher war es ein Paket pro loop()-Durchlauf;
	// das kostete pro Frame mehrere Durchlaeufe, sodass nur etwa jeder zweite
	// Frame des Sensors abgeholt wurde. Die Obergrenze verhindert, dass ein
	// Datenstau loop() lange blockiert.
	for (int i = 0; i < 8; i++) {
		if (!_read_packet()) break;
	}
}
bool Kld7::_read_packet() {
	// Liest hoechstens ein Paket. true = ein Paket wurde verarbeitet (es kann
	// ein weiteres folgen), false = im Moment nichts (mehr) zu tun.
	//
	// Header und Payload getrennt einlesen: Der Header wird gemerkt, bis die
	// komplette Payload im UART-Puffer liegt. So blockiert read_array() nie auf
	// noch nicht empfangene Bytes.
	// Schutz gegen Desynchronisation (z.B. verspaetete Antwort nach Timeout):
	//  - nur bekannte Kommandos akzeptieren,
	//  - Laenge auf das Datenblatt-Maximum begrenzen (PDAT 96 Byte = 12 Targets),
	//    deutlich unter dem UART-Empfangspuffer - sonst koennte available() die
	//    Laenge nie erreichen und die Schleife haengt dauerhaft,
	//  - wartet die Payload laenger als TIMEOUT_INTERVAL, Header verwerfen.
	// In allen Faellen: UART-Puffer leeren und neu aufsetzen. Die Request-Logik
	// in loop() fordert nach TIMEOUT_INTERVAL automatisch neue Daten an.
	char* command = _head_command;
	uint32_t& length = _head_length;
	if (!_have_head) {
		if (available() < 8) return false;
		uint8_t head[8];
		read_array(head, 8);
		memcpy(command, head, 4);
		command[4] = '\0';
		length = head[4] | (head[5]<<8) | (head[6]<<16) | ((uint32_t)head[7]<<24);
		bool known = strcmp(command, "RESP")==0 || strcmp(command, "PDAT")==0 ||
			strcmp(command, "TDAT")==0 || strcmp(command, "DDAT")==0 || strcmp(command, "DONE")==0;
		if (!known || length > MAX_PAYLOAD) {
			ESP_LOGE(TAG, "Ungueltiger Header %02X %02X %02X %02X, Laenge %lu - verwerfe UART-Puffer.", head[0], head[1], head[2], head[3], (unsigned long) length);
			while (available() > 0) read();
			return false;
		}
		_have_head = true;
		_head_ts = millis();
	}
	if (available() < (int)length) {
		if (millis() - _head_ts > (unsigned long) TIMEOUT_INTERVAL) {
			ESP_LOGE(TAG, "Payload fuer %s (%lu Byte) unvollstaendig - verwerfe UART-Puffer.", command, (unsigned long) length);
			while (available() > 0) read();
			_have_head = false;
		}
		return false;
	}
	_have_head = false;
	uint8_t payload[MAX_PAYLOAD];
	if (length > 0) read_array(payload, length);

	if (strcmp(command, "RESP")==0) {
		if (length != 1) {
			ESP_LOGE(TAG, "Received RESP with wrong payload length (expected: 1, was: %lu)", (unsigned long) length);
			return true;
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
	} else if (strcmp(command, "PDAT")==0) {
		// Nur merken. Veroeffentlicht wird in _finish_frame(), wenn die
		// Frame-Nummer bekannt ist.
		_last_pdat_targets.clear();
		int num_targets = length / 8;
		for (int i = 0; i < num_targets; i++) {
			_last_pdat_targets.push_back(RawRadarEvent(payload + i*8, 8, _invert_angle));
		}
		_pdat_pending = true;
	} else if (strcmp(command, "TDAT")==0) {
		if (length != 8 && length != 0) {
			ESP_LOGE(TAG, "Received TDAT with wrong payload length (expected: 0 or 8, was: %lu)", (unsigned long) length);
			return true;
		}
		_last_raw = RawRadarEvent(payload, length, this->_invert_angle);
		_tdat_pending = true;
		if (!_use_done) _finish_frame(++_local_frame_no);
	} else if (strcmp(command, "DONE")==0) {
		if (length != 4) {
			ESP_LOGE(TAG, "Received DONE with wrong payload length (expected: 4, was: %lu)", (unsigned long) length);
			return true;
		}
		uint32_t frame_no = payload[0] | (payload[1]<<8) | (payload[2]<<16) | ((uint32_t)payload[3]<<24);
		_finish_frame(frame_no);
	} else {
		ESP_LOGW(TAG, "Received unexpected command %s. Ignoring.", command);
	}
	return true;
}
void Kld7::_finish_frame(uint32_t frame_no) {
	// Ein Frame ist komplett (PDAT + TDAT + Frame-Nummer). Erst den naechsten
	// Frame anfordern, dann veroeffentlichen: Der Sensor braucht die Anfrage
	// moeglichst frueh, das Veroeffentlichen kostet Zeit.
	_timeout_count = 0;
	_done_missing = 0;
	_request_data();

	// Statistik: Die Frame-Nummer des Sensors zaehlt jeden gemessenen Frame.
	// Eine Luecke heisst: Diese Frames hat der ESP nicht abgeholt.
	_stat_read++;
	if (_use_done) {
		if (_have_last_frame_no && frame_no > _last_frame_no) _stat_missed += frame_no - _last_frame_no - 1;
		_last_frame_no = frame_no;
		_have_last_frame_no = true;
	}
	unsigned long now = millis();
	if (!_stat_started) {
		_stat_started = true;
		_stat_window_start = now;
	}
	if (now - _stat_window_start >= (unsigned long) STATISTICS_INTERVAL) {
		if (_use_done) {
			ESP_LOGI(TAG, "Frames in den letzten %d s: %lu abgeholt, %lu verpasst.", STATISTICS_INTERVAL / 1000, (unsigned long) _stat_read, (unsigned long) _stat_missed);
		} else {
			ESP_LOGI(TAG, "Frames in den letzten %d s: %lu abgeholt (ohne Sensor-Frame-Nummer).", STATISTICS_INTERVAL / 1000, (unsigned long) _stat_read);
		}
		_stat_read = 0;
		_stat_missed = 0;
		_stat_window_start = now;
	}

	if (!_pdat_pending) _last_pdat_targets.clear();  // kein PDAT in diesem Frame: keine alten Targets weiterverwenden
	_publish_pdat(frame_no);
	bool have_tdat = _tdat_pending;
	_pdat_pending = false;
	_tdat_pending = false;
	if (!have_tdat) return;

	if (_last_raw.detection) {
		if (_raw_speed_sensor != NULL) _raw_speed_sensor->publish_state(_last_raw.speed);
		if (_raw_angle_sensor != NULL) _raw_angle_sensor->publish_state(_last_raw.angle);
		if (_raw_distance_sensor != NULL) _raw_distance_sensor->publish_state(_last_raw.distance);
		if (_raw_magnitude_sensor != NULL) _raw_magnitude_sensor->publish_state(_last_raw.magnitude);
		if (_raw_direction_sensor != NULL) _raw_direction_sensor->publish_state(_last_raw.speed > 0);
		if (_raw_json_sensor != NULL) {
			// "f" = Frame-Nummer, identisch mit "f" im PDAT-JSON desselben Frames.
			char buff[96];
			snprintf(buff, sizeof(buff), "{\"f\":%lu,\"speed\":%.1f,\"angle\":%.1f,\"distance\":%d,\"magnitude\":%.1f}", (unsigned long) frame_no, _last_raw.speed, _last_raw.angle, _last_raw.distance, _last_raw.magnitude);
			_raw_json_sensor->publish_state(buff);
		}
	}
	if (_raw_detection_sensor != NULL) _raw_detection_sensor->publish_state(_last_raw.detection);

	_process_filtered_detection();
	_process_detection();
}
void Kld7::_publish_pdat(uint32_t frame_no) {
	int num_targets = _last_pdat_targets.size();
	if (_num_targets_sensor != NULL && num_targets != _last_num_targets) {
		_num_targets_sensor->publish_state(num_targets);
		_last_num_targets = num_targets;
	}
	// Diagnose-JSON: kompakte 1-Buchstaben-Keys (d/s/a/m), aufgeteilt auf bis
	// zu 3 Sensoren a 5 Targets, weil ein einzelner TextSensor-State von HA
	// bei 255 Zeichen gekappt wird (auf "unknown" faellt).
	// Targets 1-5 -> _pdat_json_sensor, 6-10 -> _pdat_json_sensor_2,
	// 11-15 -> _pdat_json_sensor_3 (der Sensor liefert hoechstens 12).
	// Jeder Chunk traegt die Frame-Nummer "f" und die Gesamtzahl "n". Damit
	// laesst sich im Log pruefen, ob drei Chunks zum selben Frame gehoeren und
	// ob Frames fehlen.
	// Reihenfolge 3 -> 2 -> 1: Der HA-CSV-Logger triggert auf Chunk 1 und liest
	// dabei Chunk 2 und 3 mit. Die API uebertraegt der Reihe nach, also muessen
	// 2 und 3 vor 1 veroeffentlicht werden, sonst loggt HA dort den Vorframe.
	const int TARGETS_PER_CHUNK = 5;
	text_sensor::TextSensor* chunk_sensors[3] = {_pdat_json_sensor, _pdat_json_sensor_2, _pdat_json_sensor_3};
	for (int chunk = 2; chunk >= 0; chunk--) {
		text_sensor::TextSensor* sens = chunk_sensors[chunk];
		if (sens == NULL) continue;
		int start = chunk * TARGETS_PER_CHUNK;
		if (start >= num_targets) {
			// Fuer diesen Chunk gibt es keine Targets: einmalig auf ein leeres,
			// aber valides JSON-Objekt setzen, statt den letzten (veralteten)
			// Wert stehen zu lassen. Danach nicht bei jedem Frame wiederholen -
			// bei voller Frame-Rate waeren das im Ruhezustand ueber 50 nutzlose
			// Nachrichten pro Sekunde, HA verwirft identische States ohnehin.
			if (!_pdat_chunk_empty[chunk]) {
				sens->publish_state("{}");
				_pdat_chunk_empty[chunk] = true;
			}
			continue;
		}
		_pdat_chunk_empty[chunk] = false;
		int end = start + TARGETS_PER_CHUNK;
		if (end > num_targets) end = num_targets;

		char buff[260];
		int pos = snprintf(buff, sizeof(buff), "{\"f\":%lu,\"n\":%d,\"t\":[", (unsigned long) frame_no, num_targets);
		bool truncated = false;
		for (int i = start; i < end; i++) {
			RawRadarEvent& target = _last_pdat_targets[i];
			// Sicherheitsmarge: bei worst-case Werten passt ein Target-Objekt
			// in ca. 45 Zeichen. Bricht die verbleibende Pufferkapazitaet das,
			// wird abgebrochen statt der Puffer zu ueberlaufen.
			if (pos > (int)sizeof(buff) - 50) { truncated = true; break; }
			pos += snprintf(buff+pos, sizeof(buff)-pos,
				"%s{\"d\":%d,\"s\":%.2f,\"a\":%.2f,\"m\":%.2f}",
				(i>start ? "," : ""), target.distance, target.speed, target.angle, target.magnitude);
		}
		snprintf(buff+pos, sizeof(buff)-pos, truncated ? "...]}" : "]}");
		sens->publish_state(buff);
		ESP_LOGV(TAG, "PDAT[%d]: %s", chunk+1, buff);
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
