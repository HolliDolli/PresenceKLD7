#include "radarevent.h"

namespace esphome {
namespace kld7 {
		
RawRadarEvent::RawRadarEvent(uint8_t* data, uint32_t length, bool invert_angle) {
	if (length!=8) {
		detection = false;
	} else {
		distance = data[0] | (data[1]<<8);
		speed = 0.01 * (int16_t)(data[2] | (data[3]<<8));
		angle = (invert_angle ? -0.01 : 0.01) * (int16_t)(data[4] | (data[5]<<8));

		magnitude = 0.01 * (uint16_t)(data[6] | (data[7]<<8));
		detection = true;
	}
	timestamp = millis();
}

}
}