#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "params.h"
#include "dac.h"
#include "servo.h"

#define I_FLOOR   0.01f     // nA; keeps log10 finite (one ADC count is 0.008 nA)
#define LOG_SIZE  4096

struct log_sample {
    uint32_t n;
    uint16_t raw;
    float i;
    float err;
    float z;
};

static IntervalTimer timer;
static float dt, alpha;

static volatile int mode = MODE_HOLD;
static volatile float z;          // nm, 0 = fully retracted
static volatile float integ;      // integrator state, nm
static volatile float i_filt;     // nA
static volatile double z_sum;
static volatile uint32_t z_cnt;

static volatile uint16_t dac_req[4];
static uint16_t dac_last[4];      // all 0 after the reset in dac_init

static struct log_sample logbuf[LOG_SIZE];
static volatile uint32_t log_head, log_tail, log_lost;

static float clampz(float v)
{
    if (v < 0) return 0;
    if (v > Z_RANGE_NM) return Z_RANGE_NM;
    return v;
}

static void tick(void)
{
    static uint32_t n, logc;

    int raw = analogRead(PIN_ADC);
    float i = raw * NA_PER_COUNT;
    i_filt += alpha * (i - i_filt);
    float err = log10f(fmaxf(i_filt, I_FLOOR) / par.sp);   // decades, roughly angstroms

    if (mode == MODE_FEEDBACK) {
        integ = clampz(integ - par.ki * err * dt);
        z = clampz(integ - par.kp * err);
    } else if (mode == MODE_SWEEP) {
        if (i_filt >= par.contact) {
            integ = z;
            mode = MODE_FEEDBACK;
        } else if (z >= Z_RANGE_NM) {
            mode = MODE_HOLD;
        } else {
            z = clampz(z + par.sweep * dt);
        }
    }

    dac_req[DAC_Z] = (uint16_t)(z * (65535.0f / Z_RANGE_NM) + 0.5f);
    for (int ch = 0; ch < 4; ch++) {
        if (dac_req[ch] != dac_last[ch]) {
            dac_write(ch, dac_req[ch]);
            dac_last[ch] = dac_req[ch];
        }
    }

    z_sum += z;
    z_cnt++;

    if (par.logn >= 1 && ++logc >= par.logn) {
        logc = 0;
        if (log_head - log_tail < LOG_SIZE) {
            struct log_sample *s = &logbuf[log_head % LOG_SIZE];
            s->n = n;
            s->raw = raw;
            s->i = i_filt;
            s->err = err;
            s->z = z;
            log_head++;
        } else {
            log_lost++;
        }
    }
    n++;
}

void servo_init(void)
{
    analogReadResolution(12);
    dac_req[DAC_X] = 32768;
    dac_req[DAC_Y] = 32768;
    servo_apply();
}

void servo_apply(void)
{
    if (par.fs < 100) par.fs = 100;
    if (par.fs > 50000) par.fs = 50000;
    if (par.fc < 0) par.fc = 0;
    if (par.sp < I_FLOOR) par.sp = I_FLOOR;
    if (par.avg <= 1) par.avg = 1;
    else if (par.avg <= 4) par.avg = 4;
    else if (par.avg <= 8) par.avg = 8;
    else if (par.avg <= 16) par.avg = 16;
    else par.avg = 32;

    noInterrupts();
    dt = 1.0f / par.fs;
    alpha = par.fc > 0 ? 1.0f - expf(-2.0f * PI * par.fc * dt) : 1.0f;
    analogReadAveraging((int)par.avg);
    interrupts();

    timer.begin(tick, 1e6f / par.fs);
}

void servo_set_mode(int m)
{
    noInterrupts();
    if (m == MODE_FEEDBACK && mode != MODE_FEEDBACK)
        integ = z;
    mode = m;
    interrupts();
}

int servo_mode(void)
{
    return mode;
}

void servo_set_z(float nm)
{
    noInterrupts();
    z = clampz(nm);
    integ = z;
    interrupts();
}

float servo_z(void)
{
    return z;
}

float servo_current(void)
{
    return i_filt;
}

void servo_set_xy(float x, float y)
{
    dac_req[DAC_X] = (uint16_t)(x + 0.5f);
    dac_req[DAC_Y] = (uint16_t)(y + 0.5f);
}

// The bias channel can only go negative: 0 to -DAC_VREF / BIAS_DIVIDER.
int servo_set_bias(float mv)
{
    float code = -mv / 1000.0f * BIAS_DIVIDER / DAC_VREF * 65536.0f;
    if (code < 0 || code > 65535) return -1;
    dac_req[DAC_BIAS] = (uint16_t)(code + 0.5f);
    return 0;
}

float servo_bias(void)
{
    return -dac_req[DAC_BIAS] / 65536.0f * DAC_VREF / BIAS_DIVIDER * 1000.0f;
}

void servo_zavg_reset(void)
{
    noInterrupts();
    z_sum = 0;
    z_cnt = 0;
    interrupts();
}

float servo_zavg(void)
{
    noInterrupts();
    double s = z_sum;
    uint32_t c = z_cnt;
    interrupts();
    return c ? (float)(s / c) : servo_z();
}

// Prints queued samples as: L n raw current_nA error_decades z_nm
void servo_log_drain(void)
{
    static uint32_t lost_reported;

    for (int k = 0; k < 64; k++) {
        noInterrupts();
        if (log_tail == log_head) {
            interrupts();
            break;
        }
        struct log_sample s = logbuf[log_tail % LOG_SIZE];
        log_tail++;
        interrupts();
        Serial.printf("L %lu %u %.4f %.4f %.3f\n",
                      (unsigned long)s.n, s.raw, s.i, s.err, s.z);
    }
    if (log_lost != lost_reported) {
        lost_reported = log_lost;
        Serial.printf("log lost %lu\n", (unsigned long)lost_reported);
    }
}
