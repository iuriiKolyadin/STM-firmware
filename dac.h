#pragma once
#include <stdint.h>

void dac_init(void);
void dac_write(int ch, uint16_t code);
