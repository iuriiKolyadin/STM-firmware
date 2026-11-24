#pragma once

// The servo interrupt runs at par.fs and owns the ADC and all DAC writes.

enum { MODE_HOLD, MODE_FEEDBACK, MODE_SWEEP };

void servo_init(void);
void servo_apply(void);         // call after changing par

void servo_set_mode(int mode);
int servo_mode(void);
void servo_set_z(float nm);     // only meaningful in MODE_HOLD
float servo_z(void);
float servo_current(void);      // filtered, nA

void servo_set_xy(float x, float y);   // DAC codes
int servo_set_bias(float mv);          // returns -1 if out of range
float servo_bias(void);

void servo_zavg_reset(void);
float servo_zavg(void);         // mean Z since last reset, nm

void servo_log_drain(void);
