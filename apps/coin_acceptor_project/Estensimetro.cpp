#include <stdio.h>
#include "Estensimetro.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

Estensimetro::Estensimetro(uint8_t gp){
    xGP = gp;
    adc_init();
    adc_gpio_init(xGP);
    channel = xGP-26;
}

Estensimetro::~Estensimetro()
{
} 

uint16_t Estensimetro::getWeight()
{
    adc_select_input(channel);
    return adc_read();
}
