#include <Arduino.h>
#include "config.h"
#include "params.h"
#include "stepper.h"

static const int pins[4] = {PIN_STEP_1, PIN_STEP_2, PIN_STEP_3, PIN_STEP_4};
static const uint8_t seq[8] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xc, 0x8, 0x9};   // half steps

static int phase;
static long remaining;
static long position;
static int energized;
static uint32_t last;

static void coils(uint8_t bits)
{
    for (int k = 0; k < 4; k++)
        digitalWriteFast(pins[k], (bits >> k) & 1);
}

void stepper_init(void)
{
    for (int k = 0; k < 4; k++)
        pinMode(pins[k], OUTPUT);
    coils(0);
}

void stepper_move(long steps)
{
    remaining = steps;
}

void stepper_stop(void)
{
    remaining = 0;
}

int stepper_busy(void)
{
    return remaining != 0 || energized;
}

long stepper_position(void)
{
    return position;
}

// Coils are switched off after the last step to avoid heating the mechanism.
void stepper_update(void)
{
    if (!remaining && !energized) return;
    if (micros() - last < par.stepus) return;
    last = micros();

    if (!remaining) {
        coils(0);
        energized = 0;
        return;
    }

    int dir = remaining > 0 ? 1 : -1;
    remaining -= dir;
    position += dir;
    phase = (phase + dir * STEP_TOWARD) & 7;
    coils(seq[phase]);
    energized = 1;
}
