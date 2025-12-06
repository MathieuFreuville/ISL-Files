#ifndef _MOTORS_H
#define _MOTORS_H

#include "hardware/pwm.h"

    /*Motor pins*/
    /*Motor pins*/
    #define AIN1 26
    #define BIN1 21
    #define AIN2 27
    #define BIN2 20

    #define PWMA_LEFT 0
    #define PWMB_RIGHT 3
    #define STBY_PIN 22

    /*Encoder measuring speed  pins*/
    #define LEFT_ENCODER 14
    #define RIGHT_ENCODER 12

    void Motors_Init();
    void gpio_callback(uint gpio, uint32_t events);

    void Motors_Control(int AIN1_value,int BIN1_value,int PWM_pin,int speed);

    void Motors_Move_Stop();
    void Motors_Move_Forward(uint16_t speed);
    void Motors_Move_Back(uint16_t speed);
    void Motors_Move_Left(uint16_t speed);
    void Motors_Move_Right(uint16_t speed);
    void Motors_Move_Left_Wheel(uint16_t speed);
    void Motors_Move_Right_Wheel(uint16_t speed);

    uint slice_num_PWMA_LEFT;
    uint slice_num_PWMB_RIGHT;
    uint Channel_PWMA_LEFT;
    uint Channel_PWMB_RIGHT;

    int Motor_Left_Encoder;
    int Motor_Right_Encoder;
          
#endif // _MOTORS_H