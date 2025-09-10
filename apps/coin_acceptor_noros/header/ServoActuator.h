#ifndef SERVO_ACTUATOR_H
#define SERVO_ACTUATOR_H

#include "pico/stdlib.h"

class ServoActuator {
	private:
		uint8_t xGP;

	public:
		ServoActuator(uint8_t gp);
		virtual ~ServoActuator();
		/**
		 * move to angle degree (between 0 and 180).
		 * @param degree
		 */
		void goDegree(float degree);
};

#endif /* SERVO_ACTUATOR_H */