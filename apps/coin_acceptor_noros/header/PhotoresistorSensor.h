#ifndef PHOTORESISTOR_SENSOR_H
#define PHOTORESISTOR_SENSOR_H

#include "pico/stdlib.h"

class PhotoresistorSensor {
	private:
		uint8_t xGP;
		uint8_t channel;

	public:
		PhotoresistorSensor(uint8_t gp);
		virtual ~PhotoresistorSensor();
		/**
		 * @return an unsigned 16-bit integer indicating the light read by the sensor
		 *  */ 
		uint16_t getLight();
};

#endif /* PHOTORESISTOR_SENSOR_H */