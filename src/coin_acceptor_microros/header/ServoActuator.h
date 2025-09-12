#ifndef SERVO_ACTUATOR_H
#define SERVO_ACTUATOR_H

#include "pico/stdlib.h"

/**
 * @brief Servomotor class
 */
class ServoActuator {
	private:
		/**
		 * @brief GPIO pin of the servomotor
		 */
		uint8_t xGP;

	public:
		/**
		 * @param gp GPIO pin of the servomotor
		 */
		ServoActuator(uint8_t gp);
		virtual ~ServoActuator();
		/**
		 * @brief move to angle degree (between 0 and 180).
		 * @param degree angle to get to
		 */
		void goDegree(float degree);
};

#endif /* SERVO_ACTUATOR_H */