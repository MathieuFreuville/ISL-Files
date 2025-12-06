#ifndef _WIFI_SERVER_H
#define _WIFI_SERVER_H


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"         // Pico SDK standard library
#include "pico/cyw43_arch.h"     // CYW43 WiFi chip driver
#include "lwip/tcp.h"            // Lightweight IP TCP stack
#include "lwip/pbuf.h"           // Packet buffer management
#include "lwip/netif.h"          // Network interface
#include "hardware/watchdog.h"   // Hardware watchdog timer
#include "pico/multicore.h"      // Multi-core support


#define TCP_PORT 8080                          // TCP server listening port
#define WIFI_SSID "Proximus-Home-4B80"        // WiFi network name (SSID)
#define WIFI_PASSWORD "w7hyuyfmpcpsr"         // WiFi network password
#define ACQUISITION_BUFFER_SIZE 100  // Maximum packets in circular buffer
#define DATA_STRING_SIZE 128          // Maximum size of each data packet string




float acq_float1 = 0.0f;  
float acq_float2 = 0.0f;  
float acq_float3 = 0.0f;  

int acq_int1 = 0;    
int acq_int2 = 0;    
int acq_int3 = 0;  

volatile int Telnet_Kp = 0;  // Proportional gain (modifiable via TELNET_KP command)
volatile int Telnet_Kd = 0;  // Derivative gain (modifiable via TELNET_KD command)
volatile int Telnet_Ki = 0;  // Integral gain (modifiable via TELNET_KI command)


typedef struct {
    char buffer[ACQUISITION_BUFFER_SIZE][DATA_STRING_SIZE]; 
    volatile int write_index;  // Index where Core 0 writes next data
    volatile int read_index;   // Index where Core 1 reads next data
    volatile int count;        // Number of data currently in buffer
} CircularBuffer;

// Global circular buffer 
CircularBuffer acq_buffer = {0};  


volatile bool acquisition_active = false;           // TRUE when client request data
volatile bool acquisition_was_active = false;       // Previous state of acquisition
volatile int acquisition_packets_requested = 0;     // Number of data client wants
volatile int acquisition_packets_sent = 0;          // Number of data already sent
volatile struct tcp_pcb *acquisition_pcb = NULL;    // TCP connection for sending data
volatile float Acquisition_Elapsed_Time = 0;        // Used to store elapsed time

static volatile uint32_t connection_counter = 0;   // Total connections since startup
static volatile uint32_t next_connection_id = 1;   // Unique ID for next connection


bool buffer_push(const char *data);
bool buffer_pop(char *data, int max_len);
bool buffer_has_data(void) ;

typedef struct {
    struct tcp_pcb *pcb;        // lwIP protocol control block
    uint32_t connection_id;     // Unique identifier for this connection
    bool is_valid;              // Flag to prevent use-after-free bugs
    char recv_buffer[512];      // Buffer for accumulating received command characters
    int recv_len;               // Current length of data in recv_buffer
} tcp_state_t;


static void close_conn(tcp_state_t *state);
static void tcp_err_callback(void *arg, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *client_pcb, err_t err) ;
static bool init_server(void) ;

#endif // _WIFI_SERVER_H