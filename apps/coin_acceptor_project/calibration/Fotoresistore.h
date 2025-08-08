#ifndef FOTORESISTORE_SRC_FOTORESISTORE_H_
#define FOTORESISTORE_SRC_FOTORESISTORE_H_

#include "pico/stdlib.h"

class Fotoresistore {
public:
	Fotoresistore(uint8_t gp);
	virtual ~Fotoresistore();
	uint16_t getLight();

private:
	uint8_t xGP = 0; // 32 -> GP27
    uint8_t channel;
};

#endif /* FOTORESISTORE_SRC_FOTORESISTORE_H_ */