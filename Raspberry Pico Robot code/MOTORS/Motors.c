#include <stdio.h>
#include "pico/stdlib.h"
#include "Motors.h"

void Motors_Init()
{
    gpio_init(AIN1);
    gpio_init(BIN1);
    gpio_init(AIN2);
    gpio_init(BIN2);
    gpio_init(STBY_PIN);  
    gpio_set_dir(AIN1, GPIO_OUT);
    gpio_set_dir(BIN1, GPIO_OUT);
    gpio_set_dir(AIN2, GPIO_OUT);
    gpio_set_dir(BIN2, GPIO_OUT);
    gpio_set_dir(STBY_PIN, GPIO_OUT);
    // gpio_pull_down(AIN2);
    // gpio_pull_down(BIN2);
    gpio_pull_up(STBY_PIN);
    gpio_put(STBY_PIN,1);   
    gpio_set_function(PWMA_LEFT, GPIO_FUNC_PWM);  
    gpio_set_function(PWMB_RIGHT, GPIO_FUNC_PWM);
    slice_num_PWMA_LEFT = pwm_gpio_to_slice_num(PWMA_LEFT);
    slice_num_PWMB_RIGHT = pwm_gpio_to_slice_num(PWMB_RIGHT);
    Channel_PWMA_LEFT = pwm_gpio_to_channel(PWMA_LEFT);
    Channel_PWMB_RIGHT = pwm_gpio_to_channel(PWMB_RIGHT);

    pwm_config MyPWMconfig = pwm_get_default_config();

    pwm_config_set_clkdiv_int(&MyPWMconfig, 5);        // Measured frequency 5 = 457 Hz 3=763Hz 2=1,14kHz
    pwm_init(slice_num_PWMA_LEFT, &MyPWMconfig, true);
    pwm_init(slice_num_PWMB_RIGHT, &MyPWMconfig, true);

    gpio_init(LEFT_ENCODER);
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(LEFT_ENCODER,GPIO_IN);
    gpio_set_dir(RIGHT_ENCODER,GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_pull_up(RIGHT_ENCODER);

    /*
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true, &gpio_callback); 
    */

    Motor_Left_Encoder = 0;
    Motor_Right_Encoder = 0; 
}

void gpio_callback(uint gpio, uint32_t events) 
{

    // printf("Interrupt raised \n");

    if (gpio == LEFT_ENCODER)
        {
            //printf("Interrupt raised: Left \n");
            Motor_Left_Encoder++;
        }
    else if (gpio == RIGHT_ENCODER)
        {
            //printf("Interrupt raised: Right \n");
            Motor_Right_Encoder++;
        }
    else
        {
            printf("Interrupt raised: Unknown \n");
        }   
}

void Motors_Control(int AIN1_value,int BIN1_value,int PWM_pin,int speed)
{
     
}

void Motors_Move_Stop()
{
    gpio_put(AIN1,0);
    gpio_put(BIN1,1);
    pwm_set_gpio_level(PWMA_LEFT, 0);
    pwm_set_gpio_level(PWMB_RIGHT, 0);        
}

void Motors_Move_Forward(uint16_t speed)
{
    gpio_put(AIN1,1);
    gpio_put(BIN1,1);
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,speed);
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,speed);

    // pwm_set_both_levels(slice_num,speed16,speed16);
    // pwm_set_gpio_level(PWMA_LEFT, speed16);
    // pwm_set_gpio_level(PWMB_RIGHT, speed16);      
}

void Motors_Move_Back(uint16_t speed)
{
    gpio_put(AIN1,0);
    gpio_put(BIN1,0);
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,speed);
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,speed);    
}

void Motors_Move_Left(uint16_t speed)
{
    gpio_put(AIN1,1);
    gpio_put(BIN1,1);
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,speed);   
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,0); 
}

void Motors_Move_Right(uint16_t speed)
{
    gpio_put(AIN1,1);
    gpio_put(BIN1,1);
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,0);   
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,speed);     
}

void Motors_Move_Right_Wheel(uint16_t speed)
{
    // Turn right wheel forward, helps identifying left/Right Forward/Backward
    gpio_put(AIN1,1);
    gpio_put(AIN2,0);

    gpio_put(BIN1,0);
    gpio_put(BIN2,0);
 
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,speed);   
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,0);    
}

void Motors_Move_Left_Wheel(uint16_t speed)
{
    // Turn right wheel forward, helps identifying left/Right Forward/Backward
    gpio_put(AIN1,0);
    gpio_put(AIN2,0);

    gpio_put(BIN1,0);
    gpio_put(BIN2,1);
 
    pwm_set_chan_level(slice_num_PWMA_LEFT,Channel_PWMA_LEFT,speed);   
    pwm_set_chan_level(slice_num_PWMB_RIGHT,Channel_PWMB_RIGHT,speed);   
}