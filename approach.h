#pragma once

// Repeats: retract Z, step the motor less than the Z range, sweep Z out
// until current is found. Ends in feedback.
void approach_start(void);
void approach_stop(void);
int approach_busy(void);
void approach_update(void);
