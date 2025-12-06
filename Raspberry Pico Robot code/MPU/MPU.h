#ifndef _MPU_H
#define _MPU_H

#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h> 
#include "hardware/i2c.h"


#define I2C_PORT i2c0

// Constants
#define DEG_TO_RAD_FACTOR 0.017453

// Scale modifiers
enum Enu_Accel_Scale_Modifiers{
ACCEL_SCALE_MODIFIER_2G = 16384,
ACCEL_SCALE_MODIFIER_4G = 8192,
ACCEL_SCALE_MODIFIER_8G = 4096,
ACCEL_SCALE_MODIFIER_16G = 2048
};

// !! Gyro scale modifiers are not integer values, we can't use an enumeration
#define GYRO_SCALE_MODIFIER_250DEG 131.0
#define GYRO_SCALE_MODIFIER_500DEG 65.5
#define GYRO_SCALE_MODIFIER_1000DEG 32.8
#define GYRO_SCALE_MODIFIER_2000DEG 16.4

// Pre-defined ranges
enum Enu_Accel_Range{
ACCEL_RANGE_2G = 0x00,
ACCEL_RANGE_4G = 0x08,
ACCEL_RANGE_8G = 0x10,
ACCEL_RANGE_16G = 0x18
};

enum Enu_Gyro_Ranges{
GYRO_RANGE_250DEG = 0x00,
GYRO_RANGE_500DEG = 0x08,
GYRO_RANGE_1000DEG = 0x10,
GYRO_RANGE_2000DEG = 0x18
};

enum Enu_Filter_BW{
FILTER_BW_256 = 0x00,
FILTER_BW_188 = 0x01,
FILTER_BW_98 = 0x02,
FILTER_BW_42 = 0x03,
FILTER_BW_20 = 0x04,
FILTER_BW_10 = 0x05,
FILTER_BW_5 = 0x06
};

// MPU-6050 Registers
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define ACCEL_XOUT0 0x3B
#define ACCEL_YOUT0 0x3D
#define ACCEL_ZOUT0 0x3F

#define TEMP_OUT0 0x41

#define GYRO_XOUT0 0x43
#define GYRO_YOUT0 0x45
#define GYRO_ZOUT0 0x47

#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define MPU_CONFIG 0x1A

#define WHO_AM_I 0x75
#define GRAVITY 9.80665
#define SAMPLING_PERIOD_US  10000
// 100 Hz   10000
// 200 Hz   5000
// 250 Hz   4000
// 400 Hz   2500

static int MPU_Adress = 0x68;

struct s_MPU6050
{
    enum Enu_Accel_Range Accelerometer_Scale;
    float Accelerometer_Scale_Modifier;

    enum Enu_Gyro_Ranges Gyro_Scale;
    float Gyro_Scale_Modifier;
    
    enum Enu_Filter_BW Filter_Bandwith;

    int16_t Accelerometers_raw_data[3];
    int16_t Gyros_raw_data[3];
    int16_t Temp_raw_data;  
    int16_t AccX_Calib_Value;
    int16_t AccY_Calib_Value;
    int16_t AccZ_Calib_Value;
    int16_t GyroX_Calib_Value;
    int16_t GyroY_Calib_Value;
    int16_t GyroZ_Calib_Value;

    float Accelerometer_X;      // 32 bits signed
    float Accelerometer_Y;
    float Accelerometer_Z;    
    float Gyro_Speed_X;      // 32 bits signed
    float Gyro_Speed_Y;
    float Gyro_Speed_Z;
    float Gyro_Displacment_X;      // 32 bits signed
    float Gyro_Displacment_Y;
    float Gyro_Displacment_Z;   
    float Gyro_Angle_X;      // 32 bits signed
    float Gyro_Angle_Y;
    float Gyro_Angle_Z;
    float Complementary_Ratio_Gyro;
    float Complementary_Ratio_Acc;   
    float Complementary_X;
    float Complementary_Y;
    float Complementary_Z;
    float Complementary_X_Radians;
    float Complementary_Y_Radians;
    float Complementary_Z_Radians;

    float Kalman_X;
    float Kalman_Y;
    float Kalman_Z;
    float K_Q_angle;      // Process noise variance for the accelerometer
    float K_Q_bias;       // Process noise variance for the gyro bias
    float K_R_measure;    // Measurement noise variance - this is actually the variance of the measurement noise
    float K_angle;        // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float K_bias;         // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float K_rate;         // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float K_P[2][2];      // Error covariance matrix - This is a 2x2 matrix

    float Sampling_Time;
    uint32_t Sampling_Start;
    uint32_t Sampling_Stop;

    float Calc_Deg_to_Rad_Factor;
};

void I2C_Read_Register(uint8_t *reg_adress, uint8_t *buffer, int lenght);
void I2C_Write_Register( uint8_t *buffer, int lenght);
int I2C_Write_And_Verify_Register( uint8_t *buffer, int lenght);

void Set_Accel_Range(enum Enu_Accel_Range Accel_Scale);
void Set_Gyro_Range(uint8_t Gyro_Scale);
void Set_Filter_Bandwith(enum Enu_Filter_BW Filter_BW);
void Set_Complementary_Filter(float Gyro_Level);
void Set_Kalman_Filter();

void Calibrate_Acceleros();
void Calibrate_Gyros();

void Get_Accel_Raw_Data();
void Get_Accel_Angles();

void Get_Gyro_Raw_Data();
void Get_Gyro_Angular_Speed();
void Get_Gyro_Angular_Displacment();
void Compute_Gyro_Angles();
void Initialize_Gyro_Angles();
void Get_Complementary();

void MPU6050_init(void);
void Get_All();

// To be finished
int Read_Local_Ranges();
int Read_Local_Modifiers();
int Read_Accel_Range();
int Read_Gyro_Range();
int Read_Filter_Range();
int Get_Temperature();
int Get_6_Axis();


#endif // _MPU_H