#ifndef _PINS_H
#define _PINS_H
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

/*I2C pins*/
#define I2C_SCL 5
#define I2C_SDA 4

void Pins_Initialize_GPIO();

#endif // _PINS_H