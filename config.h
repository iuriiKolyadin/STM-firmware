#pragma once

// Pins, from STM_full.kicad_sch. SPI0 is SCK 13, MOSI 11, MISO 12 (DAC SDO).
#define PIN_DAC_CS    10
#define PIN_DAC_RS    8
#define PIN_DAC_LDAC  6
#define PIN_ADC       A5

// Stepper on ULN2003 IN1..IN4, not on the schematic. Any free digital pins work.
#define PIN_STEP_1    2
#define PIN_STEP_2    3
#define PIN_STEP_3    4
#define PIN_STEP_4    5

// DAC8814 channels
#define DAC_X     0
#define DAC_Y     1
#define DAC_Z     2
#define DAC_BIAS  3

#define DAC_VREF     5.0f        // REF02; every channel outputs 0 to -DAC_VREF
#define DAC_SPI_HZ   10000000

#define ADC_VREF     3.3f
#define ADC_COUNTS   4096
#define TIA_OHMS     100e6f
#define NA_PER_COUNT (ADC_VREF / ADC_COUNTS / TIA_OHMS * 1e9f)

#define Z_RANGE_NM   700.0f      // Z code 65535; higher code moves the tip toward the sample
#define BIAS_DIVIDER 50.0f       // external divider after TP10
#define STEP_NM      25.0f       // tip motion per motor half step, approximate
#define STEP_TOWARD  1           // -1 if positive motor steps move the sample away from the tip
