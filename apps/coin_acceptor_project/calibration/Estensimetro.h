#ifndef ESTENSIMETRO_SRC_ESTENSIMETRO_H_
#define ESTENSIMETRO_SRC_ESTENSIMETRO_H_

#include "pico/stdlib.h"

class Estensimetro {
public:
	Estensimetro(uint8_t gp);
	virtual ~Estensimetro();
	uint16_t getWeight();

private:
	uint8_t xGP = 0; // 32 -> GP27
    uint8_t channel;
};

#endif /* ESTENSIMETRO_SRC_ESTENSIMETRO_H_ */