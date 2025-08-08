#include <stdio.h>
#include "Fotoresistore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"


Fotoresistore::Fotoresistore(uint8_t gp){
    xGP = gp;
    adc_init();
    adc_gpio_init(xGP);
    channel = xGP-26;
}

Fotoresistore::~Fotoresistore()
{
} 

uint16_t Fotoresistore::getLight()
{
    adc_select_input(channel);
    return adc_read();
}
