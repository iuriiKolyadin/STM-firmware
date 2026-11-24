#pragma once

// Runtime settings, all changeable over serial. Floats so one table can set them all.
struct params {
    float fs;       // loop sample rate, Hz
    float fc;       // low pass cutoff, Hz, 0 = off
    float avg;      // ADC hardware averaging: 1, 4, 8, 16 or 32
    float sp;       // current setpoint, nA
    float kp;       // nm per decade of current error
    float ki;       // nm per decade per second
    float logn;     // log every Nth loop sample, 0 = off
    float sweep;    // approach Z sweep rate, nm/s
    float contact;  // approach stops when current exceeds this, nA
    float steps;    // approach motor half steps per cycle
    float settle;   // approach settle time, ms
    float stepus;   // motor half step interval, us
    float px;       // scan pixels per side
    float size;     // scan width, DAC codes
    float dwell;    // scan time per pixel, us
    float slew;     // xy move speed, DAC codes per ms
};

extern struct params par;
