#include "config.h"
#include "params.h"
#include "dac.h"
#include "servo.h"
#include "stepper.h"
#include "approach.h"
#include "scan.h"
#include "command.h"

void setup()
{
    Serial.begin(115200);
    dac_init();
    stepper_init();
    servo_init();
}

void loop()
{
    command_poll();
    stepper_update();
    approach_update();
    scan_update();
    servo_log_drain();
}
