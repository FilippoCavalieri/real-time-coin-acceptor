#ifndef LM393_COUPLE_SENSOR_H
#define LM393_COUPLE_SENSOR_H

#include "pico/stdlib.h"

/**
 * @brief Manages a couple of photocoupler in order to simplify the overlap detection.
 */
class LM393CoupleSensor{
    private:
        /**
         * @brief GPIOs of the 2 photocoupler
         */
        uint8_t xGP1, xGP2;

    public:
        /**
         * @brief 
         * @param xGP1 GPIO of the first photocoupler
         * @param xGP2 GPIO of the second photocoupler
         */
        LM393CoupleSensor(uint8_t xGP1, uint8_t xGP2);
        virtual ~LM393CoupleSensor();
        /**
         * @return a boolean value saying if the money covered both sensor LEDs
         */
        bool getOverlap();
};

#endif /* LM393_COUPLE_SENSOR_H */