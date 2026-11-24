#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "dac.h"

static SPISettings spi_settings(DAC_SPI_HZ, MSBFIRST, SPI_MODE0);

void dac_init(void)
{
    pinMode(PIN_DAC_CS, OUTPUT);
    digitalWriteFast(PIN_DAC_CS, HIGH);

    // LDAC held low makes the DAC registers transparent, so each write
    // updates its output when CS goes high.
    pinMode(PIN_DAC_LDAC, OUTPUT);
    digitalWriteFast(PIN_DAC_LDAC, LOW);

    // MSB is tied low, so reset sets every channel to code 0 (0 V).
    pinMode(PIN_DAC_RS, OUTPUT);
    digitalWriteFast(PIN_DAC_RS, LOW);
    delayMicroseconds(10);
    digitalWriteFast(PIN_DAC_RS, HIGH);

    SPI.begin();
}

// Three bytes; the DAC keeps the last 18 bits: A1 A0 D15..D0.
void dac_write(int ch, uint16_t code)
{
    uint32_t word = ((uint32_t)(ch & 3) << 16) | code;

    SPI.beginTransaction(spi_settings);
    digitalWriteFast(PIN_DAC_CS, LOW);
    SPI.transfer((word >> 16) & 0xff);
    SPI.transfer((word >> 8) & 0xff);
    SPI.transfer(word & 0xff);
    digitalWriteFast(PIN_DAC_CS, HIGH);
    SPI.endTransaction();
}
