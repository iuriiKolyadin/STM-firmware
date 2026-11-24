#pragma once

// Owns the lateral position. A scan is centred on the current position.
int scan_start(void);           // 0 ok, -1 bad size, -2 feedback off
void scan_goto(float x, float y);
void scan_stop(void);
int scan_busy(void);
float scan_x(void);
float scan_y(void);
void scan_update(void);
