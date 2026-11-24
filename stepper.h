#pragma once

// Positive steps move the sample toward the tip.
void stepper_init(void);
void stepper_move(long steps);
void stepper_stop(void);
int stepper_busy(void);
long stepper_position(void);
void stepper_update(void);
