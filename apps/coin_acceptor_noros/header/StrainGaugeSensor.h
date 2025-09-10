#ifndef STRAIN_GAUGE_SENSOR_H
#define STRAIN_GAUGE_SENSOR_H

#include "pico/stdlib.h"

class StrainGaugeSensor {
	private:
		uint8_t xGP;
		uint8_t channel;
	public:
		StrainGaugeSensor(uint8_t gp);
		virtual ~StrainGaugeSensor();
		/**
		 * @return an unsigned 16-bit integer indicating the weight read by the sensor
		 */
		uint16_t getWeight();
};

#endif /* STRAIN_GAUGE_SENSOR_H */