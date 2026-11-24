#include <Arduino.h>
#include "params.h"
#include "servo.h"
#include "stepper.h"
#include "approach.h"

enum { A_IDLE, A_RETRACT, A_STEP, A_SETTLE, A_SWEEP };

static int state = A_IDLE;
static uint32_t t0;
static long cycles;

static void retract(int next)
{
    servo_set_mode(MODE_HOLD);
    servo_set_z(0);
    t0 = millis();
    state = next;
}

// The first sweep happens before any motor step.
void approach_start(void)
{
    cycles = 0;
    retract(A_SETTLE);
}

void approach_stop(void)
{
    if (state != A_IDLE && servo_mode() == MODE_SWEEP)
        servo_set_mode(MODE_HOLD);
    state = A_IDLE;
}

int approach_busy(void)
{
    return state != A_IDLE;
}

void approach_update(void)
{
    switch (state) {
    case A_RETRACT:
        if (millis() - t0 >= par.settle) {
            stepper_move((long)par.steps);
            state = A_STEP;
        }
        break;
    case A_STEP:
        if (!stepper_busy()) {
            t0 = millis();
            state = A_SETTLE;
        }
        break;
    case A_SETTLE:
        if (millis() - t0 >= par.settle) {
            servo_set_mode(MODE_SWEEP);
            state = A_SWEEP;
        }
        break;
    case A_SWEEP:
        if (servo_mode() == MODE_FEEDBACK) {
            Serial.printf("approach done %ld %.1f %.3f\n", cycles, servo_z(), servo_current());
            state = A_IDLE;
        } else if (servo_mode() == MODE_HOLD) {
            cycles++;
            Serial.printf("approach %ld %ld\n", cycles, stepper_position());
            retract(A_RETRACT);
        }
        break;
    }
}
