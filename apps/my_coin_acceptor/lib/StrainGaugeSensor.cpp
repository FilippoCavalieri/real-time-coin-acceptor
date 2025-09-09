#include "../header/StrainGaugeSensor.h"
#include "hardware/adc.h"

StrainGaugeSensor::StrainGaugeSensor(uint8_t xGP){
    this->xGP = xGP;
    channel = xGP-26;

    adc_init();
    adc_gpio_init(xGP);
}

StrainGaugeSensor::~StrainGaugeSensor(){} 

uint16_t StrainGaugeSensor::getWeight(){
    adc_select_input(channel);
    return adc_read();
}
