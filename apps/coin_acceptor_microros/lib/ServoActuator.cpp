#include "../header/ServoActuator.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdio.h>

#define ROTATE_0 700 //Rotate to 0° position
#define ROTATE_180 2300

ServoActuator::ServoActuator(uint8_t xGP) {
	this->xGP = xGP;

	gpio_init(xGP);

	//Setup up PWM t
	gpio_set_function(xGP, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGP, 0);
	uint slice_num = pwm_gpio_to_slice_num(xGP);

	// Get clock speed and compute divider for 50 hz
	uint32_t clk = clock_get_hz(clk_sys);
	uint32_t div = clk / (20000 * 50);

	// Check div is in range
	if ( div < 1 ){
		div = 1;
	}
	if ( div > 255 ){
		div = 255;
	}

	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, (float)div);

	// Set wrap so the period is 20 ms
	pwm_config_set_wrap(&config, 20000);

	// Load the configuration
	pwm_init(slice_num, &config, false);

	pwm_set_enabled(slice_num, true);
}

ServoActuator::~ServoActuator() {}

void ServoActuator::goDegree(float degree){
	if (degree > 180.0 || degree < 0){
		return;
	}

	int duty = (((float)(ROTATE_180 - ROTATE_0) / 180.0) * degree) + ROTATE_0;

	pwm_set_gpio_level(xGP, duty);
}