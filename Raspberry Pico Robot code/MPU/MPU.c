#include "MPU.h"

// Has to be put in C file
// See explanation her
// https://forums.raspberrypi.com/viewtopic.php?t=308690

struct s_MPU6050 MPU6050_Structure;
struct s_MPU6050 *MPU = &MPU6050_Structure; 

void I2C_Read_Register(uint8_t *reg_adress, uint8_t *buffer, int lenght)
{
    i2c_write_blocking(I2C_PORT, MPU_Adress,  reg_adress, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_Adress, buffer, lenght, false);
}

void I2C_Write_Register( uint8_t *buffer, int lenght)
{
    i2c_write_blocking(I2C_PORT, MPU_Adress,  buffer, lenght, true);
}

int I2C_Write_And_Verify_Register( uint8_t *buffer, int lenght)
{
    i2c_write_blocking(I2C_PORT, MPU_Adress,  buffer, lenght, true);
    sleep_ms(100);

    uint8_t comparaison_buffer[lenght];
    i2c_write_blocking(I2C_PORT, MPU_Adress,  &buffer[0], 1, true);
    i2c_read_blocking(I2C_PORT, MPU_Adress, comparaison_buffer, lenght, false);
    sleep_ms(100);

    bool ErrorFlag = false;
    for (int i=0; i < (lenght-1); i++)
    {
        // printf("Comparaison: 0x%02x \t 0x%02x \n", buffer[i+1], comparaison_buffer[i]);
        if (buffer[i+1] != comparaison_buffer[i])
        {
            ErrorFlag = true;
            break;
        }
    }
    if (ErrorFlag){
        while (true)
        {
            printf("Error in writing 0x%02x register", buffer[0]);
            sleep_ms(1000);
        }
    }
}

void Set_Accel_Range(enum Enu_Accel_Range Accel_Scale)
{
    uint8_t register_adress = ACCEL_CONFIG;
    printf("Setting the accelerometer range to: 0x%02x \n",Accel_Scale);

    // First change it to 0x00 to make sure we write the correct value afterwards
    uint8_t buffer[2] = {ACCEL_CONFIG,0x00};
    I2C_Write_And_Verify_Register(buffer,  2);

    // Write the new range to the ACCEL_CONFIG register
    buffer[1] = Accel_Scale;
    I2C_Write_And_Verify_Register(buffer,  2);

    // Update the MPU6050 structure
    MPU6050_Structure.Accelerometer_Scale = Accel_Scale;
    if (Accel_Scale == ACCEL_RANGE_2G){MPU6050_Structure.Accelerometer_Scale_Modifier = ACCEL_SCALE_MODIFIER_2G; printf("Range properly set to 2G\n");}
    else if (Accel_Scale == ACCEL_RANGE_4G){MPU6050_Structure.Accelerometer_Scale_Modifier = ACCEL_SCALE_MODIFIER_4G; printf("Range properly set to 4G\n ");}
    else if (Accel_Scale == ACCEL_RANGE_8G){MPU6050_Structure.Accelerometer_Scale_Modifier = ACCEL_SCALE_MODIFIER_8G; printf("Range properly set to 8G\n");}
    else if (Accel_Scale == ACCEL_RANGE_16G){MPU6050_Structure.Accelerometer_Scale_Modifier = ACCEL_SCALE_MODIFIER_16G; printf("Range properly set to 16G\n");}
    
    printf("MPU Structure Accelerometer_Scale set to: 0x%02x \n", MPU6050_Structure.Accelerometer_Scale);
    printf("MPU Structure Accelerometer_Scale_Modifier set to: %f \n\n", MPU6050_Structure.Accelerometer_Scale_Modifier);
}

void Set_Gyro_Range(uint8_t Gyro_Scale)
{
    uint8_t register_adress = GYRO_CONFIG;
    printf("Setting the gyroscope range to: 0x%02x \n",Gyro_Scale);

    // First change it to 0x00 to make sure we write the correct value afterwards
    uint8_t buffer[2] = {GYRO_CONFIG,0x00};
    I2C_Write_And_Verify_Register(buffer,  2);

    // Write the new range to the ACCEL_CONFIG register
    buffer[1] = Gyro_Scale;
    I2C_Write_And_Verify_Register(buffer,  2);

    // Update the MPU6050 structure
    MPU6050_Structure.Gyro_Scale = Gyro_Scale;
    if (Gyro_Scale == GYRO_RANGE_250DEG){MPU6050_Structure.Gyro_Scale_Modifier= GYRO_SCALE_MODIFIER_250DEG; printf("Range properly set to 250°/s\n");}
    else if (Gyro_Scale == GYRO_RANGE_500DEG){MPU6050_Structure.Gyro_Scale_Modifier= GYRO_SCALE_MODIFIER_500DEG; printf("Range properly set to 500°/s\n ");}
    else if (Gyro_Scale == GYRO_RANGE_1000DEG){MPU6050_Structure.Gyro_Scale_Modifier= GYRO_SCALE_MODIFIER_1000DEG; printf("Range properly set to 1000°/s\n");}
    else if (Gyro_Scale == GYRO_RANGE_2000DEG){MPU6050_Structure.Gyro_Scale_Modifier= GYRO_SCALE_MODIFIER_2000DEG; printf("Range properly set to 2000°/s\n");}
    
    printf("MPU Structure Gyro_Scale set to: 0x%02x \n", MPU6050_Structure.Gyro_Scale);
    printf("MPU Structure Gyro_Scale_Modifier set to: %f \n\n", MPU6050_Structure.Gyro_Scale_Modifier);

}

void Set_Filter_Bandwith(enum Enu_Filter_BW Filter_BW)
{
    uint8_t register_adress = MPU_CONFIG;
    printf("Setting the LP Filter BW to: 0x%02x \n",Filter_BW);

    // Keep the current EXT_SYNC_SET configuration in bits 3, 4, 5 in the MPU_CONFIG register
    uint8_t register_value ;
    I2C_Read_Register(&register_adress, &register_value,  1);
    printf("mpu_cfg_register = 0x%x  \n", register_value);
    register_value = register_value & 0x38; //0x38 = 00111000 Mask
    printf("mpu_cfg_register Masked = 0x%x \n", register_value);
    register_value = register_value | Filter_BW;
    printf("mpu_cfg_register with filter = 0x%x \n\n", register_value);

    uint8_t buffer[2] = {MPU_CONFIG,Filter_BW};
    I2C_Write_And_Verify_Register(buffer,  2);

    // Update the MPU6050 structure
    MPU6050_Structure.Filter_Bandwith = Filter_BW;
}

void Set_Complementary_Filter(float Gyro_Level)
{
    MPU->Complementary_Ratio_Gyro = Gyro_Level;
    MPU->Complementary_Ratio_Acc = (1-Gyro_Level);
    printf("Complementary filter ratios set to %f Gyro %f Accelero\n\n", MPU->Complementary_Ratio_Gyro, MPU->Complementary_Ratio_Acc);   
}

void Set_Kalman_Filter()
{
    /* We will set the variables like so, these can also be tuned by the user */
    MPU->K_Q_angle = 0.001f;
    MPU->K_Q_bias = 0.003f;
    MPU->K_R_measure = 0.03f;

    MPU->K_angle = 0.0f; // Reset the angle
    MPU->K_bias = 0.0f; // Reset bias

    MPU->K_P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    MPU->K_P[0][1] = 0.0f;
    MPU->K_P[1][0] = 0.0f;
    MPU->K_P[1][1] = 0.0f; 
}

void Calibrate_Acceleros()
{
    printf("Entering in accelerometers calibration sequence...\n");
    uint8_t register_adress = ACCEL_XOUT0;
    uint8_t Bytes[6];
    int16_t Angles[3];
    // Zeroing correction occurs on raw data
    // Reinitialize corrections
    MPU->AccX_Calib_Value = 0;
    MPU->AccY_Calib_Value = 0;
    MPU->AccZ_Calib_Value = 0;

    // Temp buffer for calculating
    float buf_x = 0;
    float buf_y = 0;
    float buf_z = 0;
    int i = 0;

    while(i<500)
        {
        I2C_Read_Register(&register_adress, Bytes,  6);
        Angles[0] = ( (Bytes[0]<<8) | Bytes[1]);
        Angles[1] = ( (Bytes[2]<<8) | Bytes[3]);
        Angles[2] = ( (Bytes[4]<<8) | Bytes[5]);
        buf_x += Angles[0];
        buf_y += Angles[1];
        buf_z += Angles[2];
        // printf("X Angle = %d\t%f\n", Angles[0],buf_x);
        sleep_ms(2);
        i++;
        }
    MPU->AccX_Calib_Value = (int16_t) (buf_x / 500);
    MPU->AccY_Calib_Value = (int16_t) (buf_y / 500);
    MPU->AccZ_Calib_Value = (int16_t) (buf_z / 500);
    
    printf("Accelerometers calibration values: X=%d\t Y=%d\t Z=%d\n \n",MPU->AccX_Calib_Value,MPU->AccY_Calib_Value,MPU->AccZ_Calib_Value );
}

void Calibrate_Gyros()
{
    printf("Entering in gyroscopes calibration sequence...\n");
    uint8_t register_adress = GYRO_XOUT0;
    uint8_t Bytes[6];
    int16_t Gyros_words[3];

    // Zeroing correction occurs on raw data
    // Reinitialize corrections
    MPU->GyroX_Calib_Value = 0;
    MPU->GyroY_Calib_Value = 0;
    MPU->GyroZ_Calib_Value = 0;

    // Temp buffer for calculating
    float buf_x = 0;
    float buf_y = 0;
    float buf_z = 0;
    int i = 0;

    while(i<500)
        {
        I2C_Read_Register(&register_adress, Bytes,  6);
        Gyros_words[0] = ( (Bytes[0]<<8) | Bytes[1]);
        Gyros_words[1] = ( (Bytes[2]<<8) | Bytes[3]);
        Gyros_words[2] = ( (Bytes[4]<<8) | Bytes[5]);
        buf_x += Gyros_words[0];
        buf_y += Gyros_words[1];
        buf_z += Gyros_words[2];
        //printf("X speed = %d\t%f\n", Gyros_words[0],buf_x);
        sleep_ms(2);
        i++;
        }
    MPU->GyroX_Calib_Value = (int16_t) (buf_x / 500);
    MPU->GyroY_Calib_Value = (int16_t) (buf_y / 500);
    MPU->GyroZ_Calib_Value = (int16_t) (buf_z / 500);
    printf("Gyros calibration values: X=%d\t Y=%d\t Z=%d\n \n",MPU->GyroX_Calib_Value,MPU->GyroY_Calib_Value,MPU->GyroZ_Calib_Value);
}

void Get_Accel_Raw_Data()
{
    uint8_t register_adress = ACCEL_XOUT0;
    uint8_t Bytes[6];
    
    I2C_Read_Register(&register_adress, Bytes,  6);
    MPU->Accelerometers_raw_data[0] = ( (Bytes[0]<<8) | Bytes[1]);
    MPU->Accelerometers_raw_data[1] = ( (Bytes[2]<<8) | Bytes[3]);
    MPU->Accelerometers_raw_data[2] = ( (Bytes[4]<<8) | Bytes[5]);
}

void Get_Accel_Angles()
{
    Get_Accel_Raw_Data();
    
    float Correced_Angle_X;
    float Correced_Angle_Y;
    float Correced_Angle_Z;

    //printf("Raw Angles: %d %d %d  \t",Angles[0],Angles[1],Angles[2]);
    Correced_Angle_X = (MPU->Accelerometers_raw_data[0]) / MPU->Accelerometer_Scale_Modifier;
    Correced_Angle_Y =( MPU->Accelerometers_raw_data[1]) / MPU->Accelerometer_Scale_Modifier;
    Correced_Angle_Z = (MPU->Accelerometers_raw_data[2]) / MPU->Accelerometer_Scale_Modifier;
    //printf("Correctd Angles: %f %f %f  \t",Correced_Angle_X,Correced_Angle_Y,Correced_Angle_Z);

    MPU6050_Structure.Accelerometer_X = atan( Correced_Angle_Y / sqrt( pow(Correced_Angle_X,2) + pow(Correced_Angle_Z,2))) * 180/M_PI;
    MPU6050_Structure.Accelerometer_Y = atan(-Correced_Angle_X / sqrt( pow(Correced_Angle_Y,2) + pow(Correced_Angle_Z,2))) * 180/M_PI;
    MPU6050_Structure.Accelerometer_Z = atan( Correced_Angle_Z / sqrt( pow(Correced_Angle_X,2) + pow(Correced_Angle_Y,2))) * 180/M_PI;
    // printf("Geometric Angles: %f %f %f  \n",MPU6050_Structure.Accelerometer_X,MPU6050_Structure.Accelerometer_Y,MPU6050_Structure.Accelerometer_Z);   
     
}

void Get_Gyro_Raw_Data()
{
    uint8_t register_adress = GYRO_XOUT0;
    uint8_t Bytes[6];
    
    I2C_Read_Register(&register_adress, Bytes,  6);
    MPU->Gyros_raw_data[0] = ( (Bytes[0]<<8) | Bytes[1]);
    MPU->Gyros_raw_data[1] = ( (Bytes[2]<<8) | Bytes[3]);
    MPU->Gyros_raw_data[2] = ( (Bytes[4]<<8) | Bytes[5]);

    
    
    // printf("Gyro Raw Data : %d %d %d  \n",    MPU->Gyros_raw_data[0],    MPU->Gyros_raw_data[1],    MPU->Gyros_raw_data[2]);   
     
}

void Get_Gyro_Angular_Speed()
{
    Get_Gyro_Raw_Data();
    MPU->Gyro_Speed_X = (MPU->Gyros_raw_data[0] - MPU->GyroX_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Speed_Y = (MPU->Gyros_raw_data[1] - MPU->GyroY_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Speed_Z = (MPU->Gyros_raw_data[2] - MPU->GyroZ_Calib_Value) / MPU->Gyro_Scale_Modifier;

    // printf("Gyro Ang. Speed : %f %f %f  \t",    MPU->Gyro_Speed_X,    MPU->Gyro_Speed_Y,    MPU->Gyro_Speed_Z);   
     
}

void Get_Gyro_Angular_Displacment()
{
    Get_Gyro_Angular_Speed();
    MPU->Sampling_Stop = MPU->Sampling_Start;
    MPU->Sampling_Start = get_absolute_time();
    MPU->Sampling_Time = ( (float) absolute_time_diff_us(MPU->Sampling_Stop, MPU->Sampling_Start) ) / 1000000  ;

    MPU->Gyro_Displacment_X = MPU->Sampling_Time * MPU->Gyro_Speed_X;
    MPU->Gyro_Displacment_Y = MPU->Sampling_Time * MPU->Gyro_Speed_Y;
    MPU->Gyro_Displacment_Z = MPU->Sampling_Time * MPU->Gyro_Speed_Z;
    // printf("Sampling Time= %f s \t",MPU->Sampling_Time);
}

void Compute_Gyro_Angles()
{
    Get_Gyro_Angular_Displacment();
    MPU->Gyro_Angle_X += MPU->Gyro_Displacment_X;
    MPU->Gyro_Angle_Y += MPU->Gyro_Displacment_Y;
    MPU->Gyro_Angle_Z += MPU->Gyro_Displacment_Z;

    // printf("Gyro Angles : %f %f %f  \n",    MPU->Gyro_Angle_X,    MPU->Gyro_Angle_Y,    MPU->Gyro_Angle_Z);
}

void Initialize_Gyro_Angles()
{
    // Initialize Gyro  Angles using accelerometer values
    Get_Accel_Angles();
    MPU->Gyro_Angle_X = MPU->Accelerometer_X;
    MPU->Gyro_Angle_Y = MPU->Accelerometer_Y;
    MPU->Gyro_Angle_Z = MPU->Accelerometer_Z;

    MPU->Complementary_X = MPU->Accelerometer_X;
    MPU->Complementary_Y = MPU->Accelerometer_Y;
    MPU->Complementary_Z = MPU->Accelerometer_Z;
    printf("Gyro Angles initialized at : %f %f %f  \n",    MPU->Gyro_Angle_X,    MPU->Gyro_Angle_Y,    MPU->Gyro_Angle_Z);
    printf("Compl Angles initialized at : %f %f %f  \n",    MPU->Complementary_X,    MPU->Complementary_Y,    MPU->Complementary_Z);

}

void Get_Complementary()
{
    Get_Accel_Angles();
    Compute_Gyro_Angles();
    MPU->Complementary_X += MPU->Gyro_Displacment_X;
    MPU->Complementary_Y += MPU->Gyro_Displacment_Y;
    MPU->Complementary_Z += MPU->Gyro_Displacment_Z;

    MPU->Complementary_X = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_X) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_X);   
    MPU->Complementary_Y = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_Y) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_Y); 
    MPU->Complementary_Z = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_Z) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_Z); 
    printf("Compl Angles : %f %f %f  \t",    MPU->Complementary_X,    MPU->Complementary_Y,    MPU->Complementary_Z);
}

void MPU6050_init(void)
{

    // Give some time to connect the terminal
    sleep_ms(3000);
    printf("Entering MPU6050 configuration...\n");
    sleep_ms(500);
    printf("Sampling Frequency is: %d Hz\n", (int) pow(10,6)/SAMPLING_PERIOD_US);
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400000);

    //Check to see if MPU is alive (reading of the who am I register)
    uint8_t MPU_Register = WHO_AM_I;
    uint8_t Register_Value[1];

    // i2c_write_blocking(I2C_PORT, MPU_Adress,  &MPU_Register, 1, true);
    // i2c_read_blocking(I2C_PORT, MPU_Adress, &Register_Value, 1, false);
    I2C_Read_Register(&MPU_Register, Register_Value,  1);

    printf("Who am I (decimal) = %u \n",Register_Value[0]);  
    printf("Who am I (Hex) = %x \n",Register_Value[0]);
    if(Register_Value[0] != 0x68)
    {
        while(true){
        printf("MPU6050 not found on I2C Bus\n\n");
        sleep_ms(1000);
        }
    } 
    else
    {
        printf("MPU6050 properly found on I2C Bus\n\n");
        sleep_ms(750);
    }

    // The MPU6050 is  in sleep mode, wee need to wake it up
    uint8_t wake_up_buffer[2];
    wake_up_buffer[0] = PWR_MGMT_1;
    wake_up_buffer[1] = 0x00;
    I2C_Write_And_Verify_Register(wake_up_buffer,  2);
    // i2c_write_blocking(I2C_PORT, MPU_Adress,  wake_up_buffer, 2, true);
    printf("Device has woken up...\n");
    sleep_ms(500);

    Set_Accel_Range(ACCEL_RANGE_2G);
    sleep_ms(500);
    Set_Gyro_Range(GYRO_RANGE_250DEG);
    sleep_ms(500);
    Set_Filter_Bandwith(FILTER_BW_5);
    sleep_ms(500);

    //Calibrate_Acceleros();
    Calibrate_Gyros();
    Initialize_Gyro_Angles();
    Set_Complementary_Filter(0.98);

    MPU->Calc_Deg_to_Rad_Factor = M_PI/180;
    sleep_ms(100);

}

void Get_All()
{
    uint8_t register_adress = ACCEL_XOUT0;
    uint8_t Bytes[14];
    
    I2C_Read_Register(&register_adress, Bytes,  14);
        
    MPU->Sampling_Stop = MPU->Sampling_Start;
    MPU->Sampling_Start = get_absolute_time();
    MPU->Sampling_Time = ( (float) absolute_time_diff_us(MPU->Sampling_Stop, MPU->Sampling_Start) ) / 1000000  ;

    MPU->Accelerometers_raw_data[0] = ( (Bytes[0]<<8) | Bytes[1]);
    MPU->Accelerometers_raw_data[1] = ( (Bytes[2]<<8) | Bytes[3]);
    MPU->Accelerometers_raw_data[2] = ( (Bytes[4]<<8) | Bytes[5]);
    MPU->Temp_raw_data = ( (Bytes[6]<<8) | Bytes[7]);
    MPU->Gyros_raw_data[0] = ( (Bytes[8]<<8) | Bytes[9]);
    MPU->Gyros_raw_data[1] = ( (Bytes[10]<<8) | Bytes[11]);
    MPU->Gyros_raw_data[2] = ( (Bytes[12]<<8) | Bytes[13]);

    //Compute accelerometers angles
    //-----------------------------
    float Correced_Angle_X;
    float Correced_Angle_Y;
    float Correced_Angle_Z;

    //printf("Raw Angles: %d %d %d  \t",Angles[0],Angles[1],Angles[2]);
    Correced_Angle_X = (MPU->Accelerometers_raw_data[0]) / MPU->Accelerometer_Scale_Modifier;
    Correced_Angle_Y =( MPU->Accelerometers_raw_data[1]) / MPU->Accelerometer_Scale_Modifier;
    Correced_Angle_Z = (MPU->Accelerometers_raw_data[2]) / MPU->Accelerometer_Scale_Modifier;
    //printf("Correctd Angles: %f %f %f  \t",Correced_Angle_X,Correced_Angle_Y,Correced_Angle_Z);

    MPU6050_Structure.Accelerometer_X = atan( Correced_Angle_Y / sqrt( pow(Correced_Angle_X,2) + pow(Correced_Angle_Z,2))) * 180/M_PI;
    MPU6050_Structure.Accelerometer_Y = atan(-Correced_Angle_X / sqrt( pow(Correced_Angle_Y,2) + pow(Correced_Angle_Z,2))) * 180/M_PI;
    MPU6050_Structure.Accelerometer_Z = atan( Correced_Angle_Z / sqrt( pow(Correced_Angle_X,2) + pow(Correced_Angle_Y,2))) * 180/M_PI;
    // printf("Geometric Angles: %f %f %f  \n",MPU6050_Structure.Accelerometer_X,MPU6050_Structure.Accelerometer_Y,MPU6050_Structure.Accelerometer_Z);   
   
    //Compute Gyro angles
    //-------------------
    
    /*
    MPU->Gyro_Speed_X = (MPU->Gyros_raw_data[0] - MPU->GyroX_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Speed_Y = (MPU->Gyros_raw_data[1] - MPU->GyroY_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Speed_Z = (MPU->Gyros_raw_data[2] - MPU->GyroZ_Calib_Value) / MPU->Gyro_Scale_Modifier;

    MPU->Gyro_Displacment_X = MPU->Sampling_Time * MPU->Gyro_Speed_X;
    MPU->Gyro_Displacment_Y = MPU->Sampling_Time * MPU->Gyro_Speed_Y;
    MPU->Gyro_Displacment_Z = MPU->Sampling_Time * MPU->Gyro_Speed_Z;
    // printf("Sampling Time= %f s \t",MPU->Sampling_Time);
    */
    MPU->Gyro_Displacment_X = MPU->Sampling_Time * (MPU->Gyros_raw_data[0] - MPU->GyroX_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Displacment_Y = MPU->Sampling_Time * (MPU->Gyros_raw_data[1] - MPU->GyroY_Calib_Value) / MPU->Gyro_Scale_Modifier;
    MPU->Gyro_Displacment_Z = MPU->Sampling_Time * (MPU->Gyros_raw_data[2] - MPU->GyroZ_Calib_Value) / MPU->Gyro_Scale_Modifier;
    // printf("Sampling Time= %f s \t",MPU->Sampling_Time);

    // MPU->Gyro_Angle_X += MPU->Gyro_Displacment_X;
    // MPU->Gyro_Angle_Y += MPU->Gyro_Displacment_Y;
    // MPU->Gyro_Angle_Z += MPU->Gyro_Displacment_Z;

    MPU->Complementary_X += MPU->Gyro_Displacment_X;
    MPU->Complementary_Y += MPU->Gyro_Displacment_Y;
    MPU->Complementary_Z += MPU->Gyro_Displacment_Z;

    MPU->Complementary_X = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_X) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_X);   
    MPU->Complementary_Y = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_Y) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_Y); 
    MPU->Complementary_Z = (MPU->Complementary_Ratio_Gyro * MPU->Complementary_Z) + (MPU->Complementary_Ratio_Acc * MPU->Accelerometer_Z); 
    // printf("Compl Angles : %f %f %f  \t",    MPU->Complementary_X,    MPU->Complementary_Y,    MPU->Complementary_Z);
    
    MPU->Complementary_X_Radians = MPU->Complementary_X * MPU->Calc_Deg_to_Rad_Factor;
    MPU->Complementary_Y_Radians = MPU->Complementary_Y * MPU->Calc_Deg_to_Rad_Factor;
    MPU->Complementary_Z_Radians = MPU->Complementary_Z * MPU->Calc_Deg_to_Rad_Factor;

    /*
    printf("Compl Angles X: %.2f %.2f \t Y:%.2f %.2f \t Z:%.2f %.2f  \n",    
        MPU->Complementary_X, MPU->Complementary_X_Radians,   
        MPU->Complementary_Y, MPU->Complementary_Y_Radians,   
        MPU->Complementary_Z, MPU->Complementary_Z_Radians);
    
    */

        

}