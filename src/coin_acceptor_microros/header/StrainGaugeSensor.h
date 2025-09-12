#ifndef STRAIN_GAUGE_SENSOR_H
#define STRAIN_GAUGE_SENSOR_H

#include "pico/stdlib.h"

/**
 * @brief Strain Gauge Sensor class
 */
class StrainGaugeSensor {
	private:
		/**
		 * @brief GPIO pin of the strain gauge
		 */
		uint8_t xGP;
		/**
		 * @brief ADC channel of the strain gauge
		 */
		uint8_t channel;
	public:
		/**
		 * @param gp GPIO pin of the strain gauge
		 */
		StrainGaugeSensor(uint8_t gp);
		virtual ~StrainGaugeSensor();
		/**
		 * @return An unsigned 16-bit integer indicating the weight read by the sensor
		 */
		uint16_t getWeight();
};

#endif /* STRAIN_GAUGE_SENSOR_H */