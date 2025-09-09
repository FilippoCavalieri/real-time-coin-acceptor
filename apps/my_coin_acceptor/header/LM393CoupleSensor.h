#ifndef LM393_COUPLE_SENSOR_H
#define LM393_COUPLE_SENSOR_H

#include "pico/stdlib.h"

class LM393CoupleSensor{
    private:
        uint8_t xGP1, xGP2;

    public:
        LM393CoupleSensor(uint8_t xGP1, uint8_t xGP2);
        virtual ~LM393CoupleSensor();
        /**
         * @return a boolean value saying if the money covered both sensor's LEDs
         */
        bool getOverlap();
};

#endif /* LM393_COUPLE_SENSOR_H */