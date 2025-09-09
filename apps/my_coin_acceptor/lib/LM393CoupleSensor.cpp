#include "../header/LM393CoupleSensor.h"
#include "hardware/gpio.h"

LM393CoupleSensor::LM393CoupleSensor(uint8_t xGP1, uint8_t xGP2){
    this->xGP1 = xGP1;
    this->xGP2 = xGP2;

    gpio_init(xGP1);
    gpio_set_dir(xGP1, GPIO_IN);
    gpio_pull_down(xGP1);

    gpio_init(xGP2);
    gpio_set_dir(xGP2, GPIO_IN);
    gpio_pull_down(xGP2);
}

LM393CoupleSensor::~LM393CoupleSensor(){}

bool LM393CoupleSensor::getOverlap(){
    return gpio_get(xGP1) && gpio_get(xGP2);
}
