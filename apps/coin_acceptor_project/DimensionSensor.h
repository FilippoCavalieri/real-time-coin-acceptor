#ifndef DIMENSION_SENSOR_H_
#define DIMENSION_SENSOR_H_

#include "pico/stdlib.h"
#include "hardware/gpio.h"

class DimensionSensor {
    private:
        uint8_t xGP1, xGP2;

    public:
        DimensionSensor(uint8_t xGP1, uint8_t xGP2);
        virtual ~DimensionSensor();

        bool getOverlap();
};

#endif