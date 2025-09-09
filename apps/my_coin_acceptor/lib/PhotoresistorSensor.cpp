#include "../header/PhotoresistorSensor.h"
#include "hardware/adc.h"

PhotoresistorSensor::PhotoresistorSensor(uint8_t xGP){
    this->xGP = xGP;
    channel = xGP-26;

    adc_init();
    adc_gpio_init(xGP);
}

PhotoresistorSensor::~PhotoresistorSensor(){} 

uint16_t PhotoresistorSensor::getLight(){
    adc_select_input(channel);
    return adc_read();
}
