#include <stdio.h>
#include "pico/stdlib.h"
#include "Pins.h"
#include "MPU.h"
#include "MPU.c" // needed to access the MPU structure
#include "Motors.h"
#include "Motors.c"
#include "Wifi_Server.h"
#include "Wifi_Server.c"
#include <math.h> 


#define EXCESSIVE_TILT 0.3 // Maximum allowed angle in radians
#define DEADBAND_HIGH 60
#define DEADBAND_LOW -60
#define DEADBAND_SLOPE 4
#define DEADBAND_SLOPE_OUT (510-DEADBAND_HIGH+DEADBAND_LOW)/(510-(DEADBAND_HIGH/DEADBAND_SLOPE)+(DEADBAND_LOW/DEADBAND_SLOPE))

float Vertical_Angle;
float Error_Angle;
float Last_Error_Angle;
float Desired_Angle;
bool Robot_Tilted;
bool Angle_is_Positive;
bool Angle_was_Positive;
bool Zero_Crossing;

int Kp;
int Ki;
int Kd;

float PID_Correction;
float PID_Kp_Correction;
float PID_Kd_Correction;
float PID_Ki_Correction;
float PID_Duty_Cycle;


 // Function that will run on Core 1 (Wifi server)
void core1_entry() {
    printf("\n=== CORE 1: NETWORK SERVER ===\n");
    
    // Initialize hardware watchdog timer (30 second timeout)
    // This will reset the system if the main loop stops running
    watchdog_enable(30000, 1);
    
    // Initialize WiFi chip (CYW43439)
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return;
    }
    
    // Enable station mode (connect to existing network)
    cyw43_arch_enable_sta_mode();
    
    // Connect to WiFi network
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                          CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connection failed\n");
        return;
    }
    
    printf("WiFi connected!\n");
    
    // Display network information
    struct netif *netif = netif_default;
    if (netif) {
        printf("\n=== NETWORK INFO ===\n");
        printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
        printf("Port: %d\n", TCP_PORT);
        printf("====================\n");
    }
    
    // Initialize TCP server
    if (!init_server()) {
        printf("Server init failed\n");
        return;
    }
    
    printf("\n=== SERVER READY ===\n");
    
    uint32_t last_status = 0;
    
    // Main network loop for Core 1
    while (true) {
        // Update watchdog to prevent system reset
        watchdog_update();
        
        // Process WiFi and TCP/IP stack events
        // This must be called regularly to handle network traffic
        cyw43_arch_poll();
        
        // Check if we have acquisition data to send
        if (acquisition_active && buffer_has_data() && acquisition_pcb != NULL) {
            char data_packet[DATA_STRING_SIZE];
            
            // Get next packet from circular buffer
            if (buffer_pop(data_packet, DATA_STRING_SIZE)) {
                // Add newline for easier parsing by client
                char packet_nl[DATA_STRING_SIZE + 2];
                snprintf(packet_nl, sizeof(packet_nl), "%s\n", data_packet);
                
                // Send packet via TCP
                err_t err = tcp_write(acquisition_pcb, packet_nl, 
                                     strlen(packet_nl), TCP_WRITE_FLAG_COPY);
                
                if (err == ERR_OK) {
                    tcp_output(acquisition_pcb);  // Flush output
                    acquisition_packets_sent++;
                    
                    // Log progress every 20 packets
                    if (acquisition_packets_sent % 20 == 0) {
                        printf("Sent: %d/%d\n", acquisition_packets_sent, 
                               acquisition_packets_requested);
                    }
                    
                    // Check if acquisition is complete
                    if (acquisition_packets_sent >= acquisition_packets_requested) {
                        // Send end marker
                        const char *end = "ACQUISITION_END\n";
                        tcp_write(acquisition_pcb, end, strlen(end), TCP_WRITE_FLAG_COPY);
                        tcp_output(acquisition_pcb);
                        
                        // Reset acquisition state
                        acquisition_active = false;
                        acquisition_pcb = NULL;
                        
                        printf("Acquisition COMPLETE: %d packets\n", acquisition_packets_sent);
                    }
                } else {
                    // Error sending data - abort acquisition
                    printf("Send error: %d\n", err);
                    acquisition_active = false;
                    acquisition_pcb = NULL;
                }
            }
        }
        
        /*
             // Print status every 10 seconds
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_status > 10000) {
            printf("Core1: Connections=%lu, Buffer=%d\n", 
                   connection_counter, buffer_get_count());
            last_status = now;
        }   
        */

        
        // Small delay to prevent busy-waiting
        //sleep_ms(5);
    }
}


int64_t MPU_alarm_callback(alarm_id_t id, void *user_data) {

    // ---------------------------------------------------      
    // Timer management
    // ---------------------------------------------------
    // Reactivate timer alarm
    add_alarm_in_us(SAMPLING_PERIOD_US, MPU_alarm_callback, NULL, false);
    uint32_t CallbackStart = get_absolute_time();

    // ---------------------------------------------------      
    // Angle & error status management
    // ---------------------------------------------------
    // Get all values from MPU6050
    Get_All();

    // Get the vertical Angle and error
    Vertical_Angle =  MPU-> Complementary_X_Radians;

    // Get Angle sign and catch sign changes
    Angle_was_Positive = Angle_is_Positive;
    Angle_is_Positive = (Vertical_Angle >= 0) ? true : false;
    Zero_Crossing = ( (Angle_is_Positive && !Angle_was_Positive) || (!Angle_is_Positive && Angle_was_Positive) );
    if (Zero_Crossing){printf("Zero_Crossing \n");}

    Last_Error_Angle = Error_Angle;
    Error_Angle = Desired_Angle - Vertical_Angle;

    // ---------------------------------------------------      
    // Excessive Tilt management
    // ---------------------------------------------------
    if ( (Vertical_Angle > EXCESSIVE_TILT) || (Vertical_Angle < -EXCESSIVE_TILT))
        {
            printf("Angle: %f \t",Vertical_Angle);
            printf(" EXCESSIVE TILT ANGLE \n");
            Robot_Tilted = 1;
            Motors_Move_Stop();
        }
    else
        {

    // ---------------------------------------------------      
    // Excessive Tilt Reset
    // ---------------------------------------------------
            // printf("Angle: %f\t Error: %f\t RobotTilted = %d \n",Vertical_Angle, Error_Angle, Robot_Tilted);
            if (Robot_Tilted)
                {
                    // In order to avoid big inrush current on motors, if the robot has tilted 
                    // it has first to come back around vertical position before we reactivate motors
                    printf("EXCESSIVE TILT ANGLE OCCURED - PLEASE PUT TO VERTICAL \n");
                    if ((Vertical_Angle < 0.02) && (Vertical_Angle > -0.02))
                        {Robot_Tilted = false;}

                }
    // ---------------------------------------------------      
    // PID calculation
    // ---------------------------------------------------
            else 
                {
                    PID_Kp_Correction = (Kp * Error_Angle);
                    PID_Kd_Correction = (Kd * (Error_Angle - Last_Error_Angle) / MPU->Sampling_Time);
                    PID_Ki_Correction =  (Zero_Crossing == true) ? 0 : PID_Ki_Correction + ( (Ki * Error_Angle) * MPU->Sampling_Time);


                    PID_Correction = PID_Kp_Correction + PID_Kd_Correction + PID_Ki_Correction;

                    //Constrain PID between 0 and 12V
                    if (PID_Correction > 12){PID_Correction = 12;}
                    else if (PID_Correction < -12){PID_Correction = -12;}

                    //Convert to a 16 bits integer
                    // 0=0V 65535 = 12V
                    PID_Duty_Cycle = ((PID_Correction/12) * 65535);
                    //printf("Duty Cycle = %f", PID_Duty_Cycle);

    // ---------------------------------------------------      
    // Motor mangement
    // ---------------------------------------------------                 
                    if (PID_Duty_Cycle > 0) {Motors_Move_Forward((uint16_t) PID_Duty_Cycle);}
                    else if (PID_Duty_Cycle < 0) {Motors_Move_Back((uint16_t) -PID_Duty_Cycle);}
                    // printf("%f;\t%f;\t%d; \n", Vertical_Angle, PID_Duty_Cycle, (uint16_t)PID_Duty_Cycle);   
                }

        }

    // ---------------------------------------------------      
    // Update value from Python client
    // ---------------------------------------------------
        Kp = Telnet_Kp;
        Kd = Telnet_Kd;
        Ki = Telnet_Ki;

    // ---------------------------------------------------      
    // Communication management
    // ---------------------------------------------------

        // ACQUISITION STATE CHANGE DETECTION
        if (!acquisition_active && acquisition_was_active) 
        {
            printf("Remote acquisition STOPPED\n");
            acquisition_was_active = false;
        }
        
        if (acquisition_active) {

            // ACQUISITION STATE CHANGE DETECTION
            if (!acquisition_was_active)
                {
                printf("Remote acquisition STARTED\n");
                acquisition_was_active = true;
                Acquisition_Elapsed_Time = 0;   // At first sample, reset the time counter
                }
            else
            {
                // Acquisition is active but we are not at first sample
                Acquisition_Elapsed_Time += MPU->Sampling_Time;
            }
            

            
            /*
             * FORMAT DATA AS CSV STRING
             * Create a comma-separated string with all 6 values
             * Format: float1,float2,float3,int1,int2,int3
             * 
             * Example output: "12.345,23.456,34.567,100,200,255"
             */
            char data_string[DATA_STRING_SIZE];
            snprintf(data_string, DATA_STRING_SIZE, 
                     "%.3f,%.5f,%.3f,%3f,%3f,%3f",
                     Acquisition_Elapsed_Time, Vertical_Angle, PID_Correction,
                     (PID_Kp_Correction), (PID_Kd_Correction), (PID_Ki_Correction));
            
            /*
             * PUSH DATA TO CIRCULAR BUFFER
             * The data is pushed to the inter-core buffer where Core 1
             * will read it and transmit to the client via TCP.
             */
            if (buffer_push(data_string)) {
                // Successfully added to buffer
            } else {
                // Buffer is full - data will be lost!
                // This shouldn't happen if Core 1 is reading fast enough
                printf("Core0 ERROR: Buffer full! Data lost.\n");
            }
        }

    // printf("Callback_Time %f s \n",( (float) absolute_time_diff_us(get_absolute_time(), CallbackStart) ) / 1000000);

    return 0;
}

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    multicore_launch_core1(core1_entry);
    sleep_ms(30000);
    // GPIO Initialization
    Pins_Initialize_GPIO();
    MPU6050_init();
    //Motors Initialization
    Motors_Init();

    Desired_Angle = 0;
    Kp = 52; 
    Kd = 4;
    Ki = 1;

    Telnet_Kp = Kp;
    Telnet_Kd = Kd;
    Telnet_Ki = Ki;

    MPU->Sampling_Start = get_absolute_time();
    add_alarm_in_us(SAMPLING_PERIOD_US, MPU_alarm_callback, NULL, false);


        /*
        //----------------------------
        // DEADBND TEST
        //----------------------------
                int sleeptime = 500;  
        printf("Test DeadBand - Echelons PWM de %d ms avec mesure de la rotation\n", sleeptime);
        int i;
        printf("------- Montee ------- \n");

        // Start at max before being in loop (inertia)
        Motors_Move_Back(255);
        sleep_ms(1000);
        for (i = 0; i < 256; ++i)
            {
            // reset encoders as motors are already turning
            Motor_Left_Encoder = 0;
            Motor_Right_Encoder = 0;  

            Motors_Move_Back((255 - i));
            sleep_ms(sleeptime);
            printf("Backward Motor PWM: %d, Left: %d, Right: %d \n",(255 - i), Motor_Left_Encoder,Motor_Right_Encoder);
            Motor_Left_Encoder = 0;
            Motor_Right_Encoder = 0;          
            }
        Motors_Move_Stop();
        sleep_ms(sleeptime);

        for (i = 0; i < 256; ++i)
            {
            Motors_Move_Forward(i);
            sleep_ms(sleeptime);
            printf("Forward Motor PWM: %d, Left: %d, Right: %d, \n",i, Motor_Left_Encoder,Motor_Right_Encoder);
            Motor_Left_Encoder = 0;
            Motor_Right_Encoder = 0; 
            }

        printf("------- Descente ------- \n");
        for (i = 0; i < 256; ++i)
            {
            Motors_Move_Forward((255 - i));
            sleep_ms(sleeptime);
            printf("Forward Motor PWM: %d, Left: %d, Right: %d, \n",(255 - i), Motor_Left_Encoder,Motor_Right_Encoder);
            Motor_Left_Encoder = 0;
            Motor_Right_Encoder = 0;          
            }
        Motors_Move_Stop();
        sleep_ms(sleeptime);

        for (i = 0; i < 256; ++i)
            {
            Motors_Move_Back(i);
            sleep_ms(sleeptime);
            printf("Backward Motor PWM: %d, Left: %d, Right: %d \n",i, Motor_Left_Encoder,Motor_Right_Encoder);
            Motor_Left_Encoder = 0;
            Motor_Right_Encoder = 0; 
            }

        Motors_Move_Stop();
        sleep_ms(500);  
        */
      

        while (true)
        {
    /*
     printf("LEFT\n");
    Motors_Move_Left_Wheel(150);
    sleep_ms(5000);
    printf("RIGHT\n");
    Motors_Move_Right_Wheel(150);
    sleep_ms(5000);
    */

            /*
    printf("FORWARD\n");
    Motors_Move_Forward(150);
    sleep_ms(2000);
    printf("STOP\n");
    Motors_Move_Stop(150); 
    sleep_ms(2000);         
    Motors_Move_Back(150);
    printf("BACKWARD\n");
    sleep_ms(2000);
    printf("STOP\n");
    Motors_Move_Stop(150); 
    sleep_ms(2000);           
            */
     

        }
         
}
