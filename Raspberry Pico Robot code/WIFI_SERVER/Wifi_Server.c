
#include "Wifi_Server.h"



// =====================================================================================
// CIRCULAR BUFFER FUNCTIONS
// =====================================================================================


bool buffer_push(const char *data) {
    // Check if buffer is full
    if (acq_buffer.count >= ACQUISITION_BUFFER_SIZE) {
        return false;  // Buffer overflow
    }
    
    // Copy data string into buffer 
    strncpy(acq_buffer.buffer[acq_buffer.write_index], data, DATA_STRING_SIZE - 1);
    acq_buffer.buffer[acq_buffer.write_index][DATA_STRING_SIZE - 1] = '\0';  // Ensure null termination
    
    // Advance write index 
    acq_buffer.write_index = (acq_buffer.write_index + 1) % ACQUISITION_BUFFER_SIZE;
    
    // Increment 
    acq_buffer.count++;
    
    return true;
}

bool buffer_pop(char *data, int max_len) {
    // Check if buffer is empty
    if (acq_buffer.count <= 0) {
        return false;  // No data available
    }
    
    // Copy data from buffer
    strncpy(data, acq_buffer.buffer[acq_buffer.read_index], max_len - 1);
    data[max_len - 1] = '\0';  // Ensure null termination
    
    // Advance read index
    acq_buffer.read_index = (acq_buffer.read_index + 1) % ACQUISITION_BUFFER_SIZE;
    
    // Decrement 
    acq_buffer.count--;
    
    return true;
}

// Check if buffer has any data 
bool buffer_has_data(void) {
    return acq_buffer.count > 0;
}

 // Get the number of data in the buffer
int buffer_get_count(void) {
    return acq_buffer.count;
}


// =====================================================================================
// TCP CONNECTION MANAGEMENT FUNCTIONS
// =====================================================================================

static void close_conn(tcp_state_t *state) {
    // Validate state pointer
    if (!state || !state->is_valid) return;
    
    // Mark as invalid to prevent further use
    state->is_valid = false;
    
    // Close TCP connection if it exists
    if (state->pcb) {
        // Clear all callbacks to prevent lwIP from accessing freed memory
        tcp_arg(state->pcb, NULL);
        tcp_recv(state->pcb, NULL);
        tcp_err(state->pcb, NULL);
        
        // Attempt graceful close
        err_t err = tcp_close(state->pcb);
        if (err != ERR_OK) {
            // If graceful close fails, force abort
            tcp_abort(state->pcb);
        }
        state->pcb = NULL;
    }
    
    // Free the state structure memory
    free(state);
}

static void tcp_err_callback(void *arg, err_t err) {
    tcp_state_t *state = (tcp_state_t*)arg;
    if (state && state->is_valid) {
        state->is_valid = false;
        
        // If this was the acquisition connection, cancel the acquisition
        if (acquisition_pcb == state->pcb) {
            acquisition_active = false;
            acquisition_pcb = NULL;
            printf("Acquisition cancelled (connection error)\n");
        }
        
        free(state);
    }
}

//
// =====================================================================================
// TCP RECEIVE CALLBACK
// =====================================================================================

static err_t tcp_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    tcp_state_t *state = (tcp_state_t*)arg;
    
    // Validate state pointer
    if (!state || !state->is_valid) {
        if (p) pbuf_free(p);  // Free buffer to prevent memory leak
        return ERR_VAL;
    }
    
    // NULL pbuf indicates client has closed the connection
    if (!p) {
        // Cancel acquisition if this was the acquisition connection
        if (acquisition_pcb == pcb) {
            acquisition_active = false;
            acquisition_pcb = NULL;
        }
        close_conn(state);
        return ERR_OK;
    }
    
    // Acknowledge receipt of data (TCP flow control)
    tcp_recved(pcb, p->tot_len);
    
    // Extract data from packet buffer into temporary array
    char temp_recv[p->tot_len + 1];
    pbuf_copy_partial(p, temp_recv, p->tot_len, 0);
    temp_recv[p->tot_len] = '\0';
    
    // Detect and ignore Telnet protocol negotiation sequences
    // Telnet sends commands starting with 0xFF which we don't want to process
    bool is_telnet = false;
    for (int i = 0; i < p->tot_len; i++) {
        if ((uint8_t)temp_recv[i] == 0xFF) {
            is_telnet = true;
            break;
        }
    }
    
    if (is_telnet) {
        // Clear receive buffer and ignore this data
        state->recv_len = 0;
        memset(state->recv_buffer, 0, sizeof(state->recv_buffer));
    } else {
        // Process actual command data character by character
        for (int i = 0; i < p->tot_len; i++) {
            char c = temp_recv[i];
            
            // Check for end-of-line characters 
            if (c == '\n' || c == '\r') {
                // Process the complete command 
                if (state->recv_len > 0) {
                    state->recv_buffer[state->recv_len] = '\0';  // Null terminator
                    
                    char response[512];      // Buffer for response message
                    int response_len = 0;    // Length of response
                    
                    // Clean the command string 
                    char *cmd = state->recv_buffer;
                    while (*cmd && (*cmd < 'A' || (*cmd > 'Z' && *cmd < 'a') || *cmd > 'z')) 
                        cmd++;
                    
                    // Copy cleaned command to separate buffer
                    char cleaned[64];
                    int j = 0;
                    for (char *p = cmd; *p && j < 63; p++) {
                        if (*p >= 32 && *p < 127) {  // Only ASCII
                            cleaned[j++] = *p;
                        }
                    }
                    cleaned[j] = '\0';

                    
                    // ACQUISITION command 
                    if (strncasecmp(cleaned, "ACQUISITION", 11) == 0) {
                        int packets = 0;
                        // number of packets requested
                        if (sscanf(cleaned, "ACQUISITION %d", &packets) == 1 && 
                            packets > 0 && packets <= 1000) {
                            
                            if (acquisition_active) {
                                // Already acquisition 
                                snprintf(response, sizeof(response), 
                                        "ERROR: Acquisition already in progress\n");
                            } else {
                                // Start new acquisition
                                acquisition_packets_requested = packets;
                                acquisition_packets_sent = 0;
                                acquisition_active = true;  
                                acquisition_pcb = pcb;       // Save connection for sending data
                                
                                snprintf(response, sizeof(response), 
                                        "ACQUISITION_START\n%d packets will be sent\n"
                                        "Format: float1,float2,float3,int1,int2,int3\n", packets);
                                
                                printf("Acquisition START: %d packets\n", packets);
                            }
                        } else {
                            // Invalid data count
                            snprintf(response, sizeof(response), 
                                    "ERROR: ACQUISITION <1-1000>\n");
                        }
                        response_len = strlen(response);
                        
                    // TELNET_KP 
                    } else if (strncasecmp(cleaned, "TELNET_KP", 9) == 0) {
                        int value = 0;
                        if (sscanf(cleaned, "TELNET_KP %d", &value) == 1) {
                            Telnet_Kp = value;  
                            snprintf(response, sizeof(response), 
                                    "TELNET_KP_OK\nTelnet_Kp = %d\n", Telnet_Kp);
                            printf("Telnet_Kp modified: %d\n", Telnet_Kp);
                        } else {
                            snprintf(response, sizeof(response), 
                                    "ERROR: TELNET_KP <value>\n");
                        }
                        response_len = strlen(response);
                        
                    // TELNET_KD c
                    } else if (strncasecmp(cleaned, "TELNET_KD", 9) == 0) {
                        int value = 0;
                        if (sscanf(cleaned, "TELNET_KD %d", &value) == 1) {
                            Telnet_Kd = value; 
                            snprintf(response, sizeof(response), 
                                    "TELNET_KD_OK\nTelnet_Kd = %d\n", Telnet_Kd);
                            printf("Telnet_Kd modified: %d\n", Telnet_Kd);
                        } else {
                            snprintf(response, sizeof(response), 
                                    "ERROR: TELNET_KD <value>\n");
                        }
                        response_len = strlen(response);
                        
                    // TELNET_KI
                    } else if (strncasecmp(cleaned, "TELNET_KI", 9) == 0) {
                        int value = 0;
                        if (sscanf(cleaned, "TELNET_KI %d", &value) == 1) {
                            Telnet_Ki = value;  
                            snprintf(response, sizeof(response), 
                                    "TELNET_KI_OK\nTelnet_Ki = %d\n", Telnet_Ki);
                            printf("Telnet_Ki modified: %d\n", Telnet_Ki);
                        } else {
                            snprintf(response, sizeof(response), 
                                    "ERROR: TELNET_KI <value>\n");
                        }
                        response_len = strlen(response);
                        
                    // STATUS command
                    } else if (strncasecmp(cleaned, "STATUS", 6) == 0) {
                        snprintf(response, sizeof(response),
                                "STATUS\n"
                                "Buffer: %d/%d\n"
                                "Acquisition: %s\n"
                                "Connections: %lu\n"
                                "Telnet_Kp: %d\n"
                                "Telnet_Kd: %d\n"
                                "Telnet_Ki: %d\n"
                                "END\n",
                                buffer_get_count(), ACQUISITION_BUFFER_SIZE,
                                acquisition_active ? "ACTIVE" : "INACTIVE",
                                connection_counter, Telnet_Kp, Telnet_Kd, Telnet_Ki);
                        response_len = strlen(response);
                        
                    // HELP command
                    } else if (strncasecmp(cleaned, "HELP", 4) == 0) {
                        snprintf(response, sizeof(response),
                                "AVAILABLE COMMANDS:\n"
                                "- ACQUISITION <n>    : Acquire n packets (max 1000)\n"
                                "- TELNET_KP <value>  : Modify Telnet_Kp\n"
                                "- TELNET_KD <value>  : Modify Telnet_Kd\n"
                                "- TELNET_KI <value>  : Modify Telnet_Ki\n"
                                "- STATUS             : Display system status\n"
                                "- HELP               : Display this help\n");
                        response_len = strlen(response);
                        
                    // Unknown command
                    } else {
                        snprintf(response, sizeof(response), 
                                "ERROR: Unknown command. Type HELP for available commands.\n");
                        response_len = strlen(response);
                    }
                    
                    // Send response to client
                    if (response_len > 0) {
                        tcp_write(pcb, response, response_len, TCP_WRITE_FLAG_COPY);
                        tcp_output(pcb);  // Flush the buffer
                    }
                }
                
                // Reset receive buffer for next command
                state->recv_len = 0;
                memset(state->recv_buffer, 0, sizeof(state->recv_buffer));
                
            // Printable character - add to receive buffer
            } else if (c >= 32 && c < 127) {
                if (state->recv_len < sizeof(state->recv_buffer) - 1) {
                    state->recv_buffer[state->recv_len++] = c;
                }
            }
        }
    }
    
    // Free the packet buffer
    pbuf_free(p);
    return ERR_OK;
}


// TCP accept callback - called when a new client connects
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    connection_counter++;
    uint32_t conn_id = next_connection_id++;
    
    printf("New connection #%lu (ID: %lu)\n", connection_counter, conn_id);
    
    // Validate connection
    if (err != ERR_OK || !client_pcb) return ERR_VAL;
    
    // Allocate state structure for this connection
    tcp_state_t *state = calloc(1, sizeof(tcp_state_t));
    if (!state) {
        tcp_close(client_pcb);
        return ERR_MEM;  // Out of memory
    }
    
    // Initialize state structure
    state->pcb = client_pcb;
    state->connection_id = conn_id;
    state->is_valid = true;
    state->recv_len = 0;
    memset(state->recv_buffer, 0, sizeof(state->recv_buffer));
    
    // Register callbacks for this connection
    tcp_arg(client_pcb, state);                    // Associate state with this PCB
    tcp_recv(client_pcb, tcp_recv_callback);       // Set receive callback
    tcp_err(client_pcb, tcp_err_callback);         // Set error callback
    
    // Send welcome message to client
    const char *welcome = "PICO_DATA_SERVER\n"
                         "Commands: ACQUISITION <n>, TELNET_KP <val>, TELNET_KD <val>, "
                         "TELNET_KI <val>, STATUS, HELP\n";
    tcp_write(client_pcb, welcome, strlen(welcome), TCP_WRITE_FLAG_COPY);
    tcp_output(client_pcb);
    
    return ERR_OK;
}


//Initialize the TCP server
static bool init_server(void) {
    // Create new TCP PCB
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) return false;
    
    // Bind to port
    if (tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT)) {
        tcp_close(pcb);
        return false;
    }
    
    // Start listening
    pcb = tcp_listen(pcb);
    if (!pcb) return false;
    
    // Register accept callback
    tcp_accept(pcb, tcp_accept_callback);
    
    printf("TCP server listening on port %d\n", TCP_PORT);
    return true;
}

