#ifndef PHOTORESISTOR_SENSOR_H
#define PHOTORESISTOR_SENSOR_H

#include "pico/stdlib.h"

/**
 * @brief Photoresistor class
 */
class PhotoresistorSensor {
	private:
		/**
		 * GPIO pin of the photoresistor
		 */
		uint8_t xGP;

		/**
		 * @brief ADC channel of the photoresistor
		 */
		uint8_t channel;

	public:

		/**
		 * @param gp GPIO pin of the photoresistor
		 */
		PhotoresistorSensor(uint8_t gp);
		virtual ~PhotoresistorSensor();
		/**
		 * @return An unsigned 16-bit integer indicating the light read by the sensor
		 *  */ 
		uint16_t getLight();
};

#endif /* PHOTORESISTOR_SENSOR_H */