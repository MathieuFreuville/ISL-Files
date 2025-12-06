#include "Pins.h"


void Pins_Initialize_GPIO()
{
sleep_ms(3000);
printf("Entering GPIO configuration...\n");

gpio_set_function ( I2C_SDA, GPIO_FUNC_I2C );
gpio_set_function ( I2C_SCL, GPIO_FUNC_I2C );
gpio_pull_up ( I2C_SDA );
gpio_pull_up ( I2C_SCL );

}