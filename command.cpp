#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "params.h"
#include "servo.h"
#include "stepper.h"
#include "approach.h"
#include "scan.h"
#include "command.h"

static const struct {
    const char *name;
    float *val;
    const char *help;
} table[] = {
    {"fs", &par.fs, "loop sample rate, Hz"},
    {"fc", &par.fc, "low pass cutoff, Hz, 0 = off"},
    {"avg", &par.avg, "ADC hardware averaging, 1 4 8 16 32"},
    {"sp", &par.sp, "setpoint, nA"},
    {"kp", &par.kp, "proportional gain, nm per decade"},
    {"ki", &par.ki, "integral gain, nm per decade per s"},
    {"log", &par.logn, "log every Nth sample, 0 = off"},
    {"sweep", &par.sweep, "approach Z sweep rate, nm/s"},
    {"contact", &par.contact, "approach contact current, nA"},
    {"steps", &par.steps, "approach half steps per cycle"},
    {"settle", &par.settle, "approach settle time, ms"},
    {"stepus", &par.stepus, "motor half step interval, us"},
    {"px", &par.px, "scan pixels per side"},
    {"size", &par.size, "scan width, DAC codes"},
    {"dwell", &par.dwell, "scan time per pixel, us"},
    {"slew", &par.slew, "xy move speed, DAC codes per ms"},
};
#define NPAR (int)(sizeof(table) / sizeof(table[0]))

static const char *mode_name[] = {"hold", "feedback", "sweep"};

static char line[96];
static int len;

static void help(void)
{
    Serial.printf("status\n");
    Serial.printf("stop                stop approach, scan and motor\n");
    Serial.printf("fb on|off           feedback; off holds Z where it is\n");
    Serial.printf("z NM                set Z when feedback is off, 0 = retracted\n");
    Serial.printf("xy X Y              move to DAC codes X Y\n");
    Serial.printf("bias MV             sample bias, 0 or negative\n");
    Serial.printf("approach            automatic approach\n");
    Serial.printf("motor N             retract Z, move N half steps, + = toward\n");
    Serial.printf("withdraw N          retract Z, move N half steps away\n");
    Serial.printf("scan                scan centred on current xy\n");
    for (int k = 0; k < NPAR; k++)
        Serial.printf("%-19s %s\n", table[k].name, table[k].help);
}

static void status(void)
{
    Serial.printf("mode %s\n", mode_name[servo_mode()]);
    Serial.printf("z %.2f\n", servo_z());
    Serial.printf("i %.4f\n", servo_current());
    Serial.printf("x %.0f\n", scan_x());
    Serial.printf("y %.0f\n", scan_y());
    Serial.printf("bias %.2f\n", servo_bias());
    Serial.printf("motor %ld\n", stepper_position());
    Serial.printf("approaching %d\n", approach_busy());
    Serial.printf("scanning %d\n", scan_busy());
    for (int k = 0; k < NPAR; k++)
        Serial.printf("%s %g\n", table[k].name, *table[k].val);
}

static int busy(void)
{
    if (approach_busy() || scan_busy() || stepper_busy()) {
        Serial.printf("err busy\n");
        return 1;
    }
    return 0;
}

static void retract(void)
{
    servo_set_mode(MODE_HOLD);
    servo_set_z(0);
}

static void execute(char *s)
{
    char *cmd = strtok(s, " \t");
    char *a = strtok(NULL, " \t");
    char *b = strtok(NULL, " \t");

    if (!cmd) return;

    if (!strcmp(cmd, "help")) {
        help();
    } else if (!strcmp(cmd, "status")) {
        status();
    } else if (!strcmp(cmd, "stop")) {
        approach_stop();
        scan_stop();
        stepper_stop();
        if (servo_mode() == MODE_SWEEP) servo_set_mode(MODE_HOLD);
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "fb") && a) {
        if (busy()) return;
        servo_set_mode(!strcmp(a, "on") ? MODE_FEEDBACK : MODE_HOLD);
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "z") && a) {
        if (busy()) return;
        if (servo_mode() != MODE_HOLD) {
            Serial.printf("err feedback on\n");
            return;
        }
        servo_set_z(atof(a));
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "xy") && b) {
        if (busy()) return;
        float x = atof(a), y = atof(b);
        if (x < 0 || x > 65535 || y < 0 || y > 65535) {
            Serial.printf("err range\n");
            return;
        }
        scan_goto(x, y);
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "bias") && a) {
        if (servo_set_bias(atof(a)) < 0) {
            Serial.printf("err range\n");
            return;
        }
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "approach")) {
        if (busy()) return;
        if (par.steps * STEP_NM > 0.8f * Z_RANGE_NM) {
            Serial.printf("err steps larger than Z range\n");
            return;
        }
        if (servo_bias() == 0) {
            Serial.printf("err bias is 0\n");
            return;
        }
        approach_start();
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "motor") && a) {
        if (busy()) return;
        retract();
        stepper_move(atol(a));
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "withdraw") && a) {
        approach_stop();
        scan_stop();
        retract();
        stepper_move(-labs(atol(a)));
        Serial.printf("ok\n");
    } else if (!strcmp(cmd, "scan")) {
        if (busy()) return;
        int r = scan_start();
        if (r == -1) Serial.printf("err scan size\n");
        else if (r == -2) Serial.printf("err feedback off\n");
        else Serial.printf("ok\n");
    } else {
        for (int k = 0; k < NPAR; k++) {
            if (strcmp(cmd, table[k].name)) continue;
            if (a) {
                *table[k].val = atof(a);
                servo_apply();
            }
            Serial.printf("%s %g\n", table[k].name, *table[k].val);
            return;
        }
        Serial.printf("err unknown command\n");
    }
}

void command_poll(void)
{
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            line[len] = 0;
            execute(line);
            len = 0;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}
