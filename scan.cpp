#include <Arduino.h>
#include <math.h>
#include "params.h"
#include "servo.h"
#include "scan.h"

#define MAX_PX 512

enum { S_IDLE, S_MOVE, S_LINE };

static int state = S_IDLE;
static int scanning;
static float x = 32768, y = 32768;   // must match servo_init
static float tx, ty, step;
static uint32_t period, t0;
static int px, row, col;
static float cx, cy, xs, ys, pitch;
static float zline[MAX_PX];

static void start_move(float to_x, float to_y, float st, uint32_t us)
{
    tx = to_x;
    ty = to_y;
    step = st;
    period = us;
    t0 = micros();
    state = S_MOVE;
}

int scan_start(void)
{
    px = (int)par.px;
    pitch = par.size / px;
    xs = x - par.size / 2;
    ys = y - par.size / 2;
    if (px < 2 || px > MAX_PX || pitch < 1) return -1;
    if (xs < 0 || ys < 0 || xs + par.size > 65535 || ys + par.size > 65535) return -1;
    if (servo_mode() != MODE_FEEDBACK) return -2;

    cx = x;
    cy = y;
    row = 0;
    scanning = 1;
    Serial.printf("scan start %d %.0f %.0f %.0f %.0f\n", px, par.size, par.dwell, cx, cy);
    start_move(xs, ys, pitch, (uint32_t)par.dwell);
    return 0;
}

void scan_goto(float gx, float gy)
{
    scanning = 0;
    start_move(gx, gy, fmaxf(par.slew, 1), 1000);
}

void scan_stop(void)
{
    scanning = 0;
    state = S_IDLE;
}

int scan_busy(void)
{
    return state != S_IDLE;
}

float scan_x(void)
{
    return x;
}

float scan_y(void)
{
    return y;
}

static void print_line(void)
{
    Serial.printf("S %d", row);
    for (int k = 0; k < px; k++)
        Serial.printf(" %.2f", zline[k]);
    Serial.printf("\n");
}

void scan_update(void)
{
    if (state == S_IDLE) return;
    if (scanning && servo_mode() != MODE_FEEDBACK) {
        Serial.printf("scan aborted\n");
        scan_stop();
        return;
    }
    if (micros() - t0 < period) return;
    t0 = micros();

    if (state == S_MOVE) {
        float dx = tx - x, dy = ty - y;
        float d = fmaxf(fabsf(dx), fabsf(dy));
        if (d <= step) {
            x = tx;
            y = ty;
        } else {
            x += dx * step / d;
            y += dy * step / d;
        }
        servo_set_xy(x, y);
        if (x != tx || y != ty) return;

        if (!scanning) {
            state = S_IDLE;
        } else if (row == px) {
            scanning = 0;
            state = S_IDLE;
            Serial.printf("scan done\n");
        } else {
            col = 0;
            servo_zavg_reset();
            state = S_LINE;
        }
        return;
    }

    // S_LINE: one dwell has passed at pixel col
    zline[col++] = servo_zavg();
    if (col < px) {
        x = xs + col * pitch;
        servo_set_xy(x, y);
        servo_zavg_reset();
        return;
    }
    print_line();
    row++;
    if (row < px)
        start_move(xs, ys + row * pitch, pitch, period);
    else
        start_move(cx, cy, pitch, period);
}
