#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "time.h"

//defines
#define UART_HELTEC_TX (33)
#define UART_HELTEC_RX (34)
#define UART_SIM_TX (27) //GSM Rx -> IO27
#define UART_SIM_RX (26) //GSM Tx -> IO2


#define UART_BUFFER_SIZE 128
#define UART_SIM_BUFFER (248)
#define UART_PORT_H (UART_NUM_1) //uart port fo the heltec
#define UART_PORT_SIM (UART_NUM_2) //uart port for the sim card module
#define TSIM_PWR (GPIO_NUM_4)

#define SENSOR_MAX_IND (3)
#define SENSOR_MIN_IND (1)
#define SENSOR_RECEIVE_TIMEOUT (60000)
#define timestamp_len  (30)
#define TXT_LEN (180) //based on the GSM character set of SIM7000G

// rtos handles
static QueueHandle_t uart_queue1;
static QueueHandle_t data_queue; 

static TaskHandle_t xProcess; 
static BaseType_t process_task; 

static TaskHandle_t xProtoSim; 
static BaseType_t pro_to_sim;

static TaskHandle_t xSimInit; 
static BaseType_t sim_init;   

static SemaphoreHandle_t xParseReady;
static SemaphoreHandle_t xSendGSM;
static TimerHandle_t xReceiveTimeout = NULL;


// private variables

char* tx_data = "Hello world from TSIM\r\n"; 
char rx_data[128];

uint8_t new_line_ctr=0;
char uart_rx_buffer[UART_BUFFER_SIZE];  //for the data sent copied from the rx_buffer
char uart_at_buffer[TXT_LEN]; //to be sent by the sim 
uint16_t uart_at_index =0; 
uint16_t uart_rx_index = 0;
char uart_rx_char;
bool parse_ready = false; //gawing binary semaphore (kapag gumagana na) 

static const char *TAG_UART = "UART_HELTEC_LOG";
static const char *TAG_SENSOR = "SENSOR_LOG"; 
static const char *TAG_SIM = "UART_SIM_LOG";

static bool sensor_timeout = false; //for the payload timer
static bool sensor_ready = false; 
static bool reTx = false; 

bool at_success= false; 

typedef struct {
    uint16_t id;
    uint8_t type;
    uint8_t index;
    char timestamp[timestamp_len]; //include timestamp 
    float temp;
    float DO;
    float pH;
    float batt;
    bool lowbatt; 
    bool retransmitted; 
    uint8_t curr_index;     
} Sensor_Data; 

Sensor_Data sensor_recent[SENSOR_MAX_IND+1]; //to hold data yet to be processed after packet retransmission 
Sensor_Data sensor_buffer[SENSOR_MAX_IND+1];
bool index_received[SENSOR_MAX_IND+1] ={false};
uint8_t received_count= 0;

//prototype functions 
void init_uart (uart_port_t uart_num, int tx, int rx, int baud,QueueHandle_t* uart_queue); 
static void uart_event_task (void* arg); 
static void process_uart_task(void* arg); 
static void parse_sensor_data (char* buffer, Sensor_Data* sensor); 
static void process_to_sim(void *arg); 
static void module_init(void *arg);
void set_text_message (uint8_t count); 
//static void send_from_sim(void* arg); 
void sensor_timeout_callback(TimerHandle_t xTimer); 
bool sendATCommand(char* command, char* expectedResponse, int timeoutMs); 
bool parse_data(uint8_t* data, size_t len, int timeout_ms , const char* word); 
void sim7000G_init (void);
void copy_buffer (Sensor_Data* original, Sensor_Data* copy); 
static void send_from_sim(void *arg);




//---------------------------------------------------------------------------------//
//start of the main code 

void app_main(void)
{
    init_uart(UART_PORT_H, UART_HELTEC_TX, UART_HELTEC_RX, 38400, &uart_queue1); //to initialize interrupt-baesd UART reception

    xTaskCreate(uart_event_task, "uart_event", 4096, NULL, 5, NULL);  
    xParseReady = xSemaphoreCreateBinary(); 
    xSendGSM = xSemaphoreCreateBinary();
     
    data_queue = xQueueCreate(1,(sizeof(Sensor_Data))*(SENSOR_MAX_IND+1)); // queue initialization


    xReceiveTimeout = xTimerCreate("Sensor-Timeout", pdMS_TO_TICKS(SENSOR_RECEIVE_TIMEOUT), pdFALSE, NULL, sensor_timeout_callback); 

    process_task = xTaskCreate(process_uart_task, "uart_process", 4096, NULL, 6, &xProcess);
    configASSERT(process_task == pdPASS); 
    
    pro_to_sim = xTaskCreate(process_to_sim, "process_task", 4096, NULL, 7, &xProtoSim); 
    configASSERT(pro_to_sim == pdPASS); 

    //for debugging only, delete after 
    //xTaskCreate(send_from_sim, "Send-TXT",2048, NULL,9, NULL); 


        
    sim_init = xTaskCreate(module_init, "Sim-Init", 4096, NULL, 8, &xSimInit); 
    configASSERT(sim_init==pdPASS); 



}

//uart-based functions and initializations 
void init_uart (uart_port_t uart_num, int tx, int rx, int baud,QueueHandle_t* uart_queue){
     uart_config_t uart_config = {
    .baud_rate =baud, 
    .data_bits = UART_DATA_8_BITS, 
    .parity = UART_PARITY_DISABLE, 
    .stop_bits = UART_STOP_BITS_1, 
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB, 
    }; 

    uart_param_config(uart_num, &uart_config); 
    uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE); 
    uart_driver_install(uart_num, 1024, 0, 20 , uart_queue,0);
    
}

static void uart_event_task (void* arg){
    uart_event_t event;
    uint8_t data[UART_BUFFER_SIZE]; 
    int new_line_ctr = 0;

    while (1) {
        if (xQueueReceive(uart_queue1, &event, portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA:
                int len = uart_read_bytes(UART_PORT_H, data, event.size, portMAX_DELAY);
                for (int i = 0; i < len; i++) {
                    if (data[i] == '\n' || data[i] == '\r') {
                        new_line_ctr++;
                        if (new_line_ctr == 4) {
                            uart_rx_buffer[uart_rx_index] = '\0';
                            new_line_ctr = 0;
                            uart_rx_index = 0;
                            xSemaphoreGive(xParseReady);
                            ESP_LOGI(TAG_SENSOR, "parse ready");  
                        }
                    } else if (uart_rx_index < UART_BUFFER_SIZE - 1) {
                        uart_rx_buffer[uart_rx_index++] = data[i];
                    }
                }
                break;
            default:
                break;
            }           
        }
    }
}


static void process_uart_task (void* arg){
   
    //redefine to tunnel data to another task instead of bathc processinign 
    // char print[10];      
    uint8_t curr_index = 0; 
   // uint8_t valid_count = 0;
    
    char*tx_ack ="type:1,ACK"; 
    char rx_data_buffer[UART_BUFFER_SIZE];
 
    memset(sensor_buffer, 0, sizeof(sensor_buffer)); 


    while (1){
        if (xSemaphoreTake(xParseReady, portMAX_DELAY)){
            memset(rx_data_buffer,'\0',UART_BUFFER_SIZE); 
            strcpy(rx_data_buffer, &uart_rx_buffer[5]);
           // ESP_LOGI(TAG_SENSOR,"%c being recognized", rx_data_buffer[5]); 
            memset(uart_rx_buffer,'\0',UART_BUFFER_SIZE);
        
       
            switch (rx_data_buffer[5]){
                case '0':
                   if (sensor_ready){

                        Sensor_Data temp_data={0}; //struct used only for this purpose; temporary data 
                        parse_sensor_data(rx_data_buffer,&temp_data); //parse payload
                        
                        if (temp_data.index >= SENSOR_MIN_IND && temp_data.index <= SENSOR_MAX_IND){
                            curr_index = temp_data.index;
                            if (!index_received[curr_index]){ //checks for duplicates only
                                sensor_buffer[curr_index] = temp_data; 
                                index_received[curr_index] = true; 
                                received_count++; 
                                ESP_LOGI(TAG_SENSOR, "Stored data index %i (count: %i)", curr_index, received_count); 
                            } else {
                                ESP_LOGW(TAG_SENSOR," Duplicate of data at %i index", curr_index); 
                            }
                            
                            if (received_count && !sensor_timeout){
                                xTimerStart(xReceiveTimeout, 0); 
                                ESP_LOGW(TAG_SENSOR, "Software timer1 has started"); 
                                sensor_timeout = true;
                            }
                            
                            if (received_count == SENSOR_MAX_IND) {
                                ESP_LOGI(TAG_SENSOR, "Batch sent to SIM Task, timer has been deactivated"); 
                                if (sensor_timeout){
                                    xTimerStop(xReceiveTimeout, 0); 
                                    sensor_timeout = false; 
                                }
                                xQueueSend(data_queue, sensor_buffer, portMAX_DELAY); 
                            // xQueueSend(count_queue,received_count, portMAX_DELAY);

                                memset(sensor_buffer, 0 , sizeof(sensor_buffer)); 
                                memset(index_received, 0 , sizeof(index_received)); 
                                received_count= 0 ;  

                                
                            }
                        } else{
                            ESP_LOGW(TAG_SENSOR, "Invalid index received: %i", temp_data.index); 
                        }

                        //based on the index values
                        memset(uart_rx_buffer, '\0', sizeof(uart_rx_buffer)); 
                    }

                    
                    break;
                case '4': 
                //insert a semaphore that is only activated by restransmit later 
                //trigger ack; 
                    vTaskDelay(2000/portTICK_PERIOD_MS); 
                    uart_write_bytes(UART_PORT_H,tx_ack,10); 
                    ESP_LOGI(TAG_UART, "Transmitted to Heltec."); 
                    sensor_ready=true; 
                    //di ko na process timestamp here hehe 
                    break; 

                case '3':
                //parse retransmitted data,
                //stick to given index, then add to the process task
                    break; 
                default:
                    ESP_LOGW(TAG_UART, "Invalid payload format."); 
                    break; 
                
            }
            memset(rx_data_buffer, '\0', UART_BUFFER_SIZE);
           
        }
    }
}

void sensor_timeout_callback (TimerHandle_t xTimer){ //parang ginawa ko lang na delay timer for sending one 
    ESP_LOGW(TAG_SENSOR, "Sending received data to the queue.");
    xQueueSend(data_queue, sensor_buffer, portMAX_DELAY);
    memset(sensor_buffer, 0 , sizeof(sensor_buffer)); 
    memset(index_received, 0 , sizeof(index_received)); 
    xTimerStop(xReceiveTimeout, 0); 
    sensor_timeout = false; 
    received_count= 0 ;  

}

//parsing and further processing of data 
 
static void parse_sensor_data (char* buffer, Sensor_Data* sensor){

    if (buffer == NULL || sensor == NULL) {
        ESP_LOGE(TAG_SENSOR, "Null pointer passed to parse_sensor_data"); 

        return;
    }

    sensor->lowbatt = 0; 
    

    char* token = strtok(buffer, ",");
	while (token != NULL) {
		char* delimiter = strchr(token, ':');
		    if (delimiter != NULL) {
			    *delimiter = '\0';
                char* key = token;
                char* value = delimiter + 1;
                
                if (strcmp(key,"temp") == 0){
                        sensor->temp = atof(value);
                    }else if (strcmp(key,"ph") == 0){
                        sensor->pH = atof(value);
                    }else if (strcmp(key,"do") == 0){
                        sensor->DO = atof(value);
                    }else if (strcmp(key,"type") == 0){
                        sensor->type = atoi(value);
                    } else if (strcmp(key,"id") == 0){
                        sensor->id= atoi(value);
                    }else if (strcmp(key,"batt") == 0){
                        sensor->batt= atof(value);
                    }else if (strcmp(key,"lowbatt")== 0){
                        sensor->lowbatt = atoi(value); //low batt transmit 
                        ESP_LOGW(TAG_SENSOR, "Low batt indicator");
                    } else if (strcmp(key,"ind")== 0){
                        sensor->index = atoi(value); 
                    } else  if (strcmp(key,"time")==0) {
                        strncpy(sensor->timestamp, value, sizeof(sensor->timestamp) - 1);
                        sensor->timestamp[sizeof(sensor->timestamp)-1] = '\0'; 
                    } else{
                        ESP_LOGW(TAG_SENSOR, "Invalid payload key.");
                        break; 
                        //do not process, 
                    }

				}
		token = strtok(NULL, ",");
    }        
    ESP_LOGI(TAG_SENSOR, "Parsed sensor data: ID=%d, Temp=%.2f, pH=%.2f, DO=%.2f, Batt=%.2f, timestamp=%s, Index=%d, Lowbatt statt=%d", sensor->id, sensor->temp, sensor->pH, sensor->DO, sensor->batt,sensor->timestamp, sensor->index, sensor->lowbatt);

}

static void process_to_sim(void *arg){
    Sensor_Data sensor_buff[SENSOR_MAX_IND+1]; 
   // char text_message[TXT_LEN]; 
    uint8_t valid_count;
    //uint8_t reTx_count; 
    char*tx_ack ="type:1,ACK"; 
    //char* retrans ="type:3";
    char index [10];
    char temp[3] ;  
  //  Sensor_Data sensor_received; 
    uint8_t ind_buffer[5]; 
 //   uint8_t index_copy = 0; 
    
 
    while (1){
            
            if (xQueueReceive(data_queue, &sensor_buff, portMAX_DELAY)){
                valid_count = 0 ;  
                
            //count valid entries ;if not, get the index aand form tphe retransmit packet 
        
                for(int i=1; i <= SENSOR_MAX_IND  ; i++){
                        if (sensor_buff[i].index != 0){
                            //ind_buffer[i] = sensor_buffer[i].index; 
                            valid_count ++;
                        }else{
                            ind_buffer[i]= 1; 
                        }
                     ESP_LOGI(TAG_SENSOR,"Expected lost indices  in the sensor buffer, %d",sensor_buff[i].index);
                     ESP_LOGI(TAG_SENSOR, "index buffer %d", ind_buffer[i]); 
                }
                
                if (valid_count >= SENSOR_MAX_IND-1){
                    ESP_LOGI(TAG_SIM, "valid count %i", valid_count); 
                    uart_write_bytes(UART_PORT_H,tx_ack,10); //send acknowledgement for valid nmumber of data 
                    ESP_LOGI(TAG_SENSOR,"ACK sent for receiving valid number of sensor data."); 
                    //proceed to averaging all 
                    //set_text_message(sensor_buffer,text_message,valid_count); //enter the function with debugging points
                    //semaphore for the text 
                    //xSemaphoreGive(xSensorReady); 
                    memset(uart_at_buffer, '\0',TXT_LEN); 
                    copy_buffer(sensor_buff,sensor_recent);
                    set_text_message(valid_count);
                    xSemaphoreGive(xSendGSM); 
        

                  //  memset(sensor_recent, 0, sizeof(sensor_recent)); 
                } else {
                    for(int i=1; i <= SENSOR_MAX_IND  ; i++){
                        sensor_recent[i]= sensor_buffer[i]; //store data from last transmit 

                        if (ind_buffer[i]){
                            memset(temp, '\0', 1);
                            sprintf(temp, "%d,", i); 
                            strcat(index, temp);
                        }
                        
                    }
                    char retransmit [30] = "type:3,ind:";
                    strcat(retransmit,index);
                    ESP_LOGI(TAG_SENSOR, "sending restrans string %s", retransmit); 
                    uart_write_bytes(UART_PORT_H,retransmit,sizeof(retransmit)); 
                    reTx = true; 
                  //  memcpy(sensor_recent, sensor_buffer, sizeof(sensor_buffer)); 
                    memset(retransmit, '\0', sizeof(retransmit));
                    memset(index, '\0', sizeof(index)); 
                    memset(ind_buffer,0, sizeof(ind_buffer)); 
                    //valid_count = 0; 
                    }
               // memset(sensor_buffer, 0, sizeof(sensor_buffer)); dont forget to uncomment
               // memset(ind_buffer, 0, sizeof(ind_buffer)); 
 
            //average all readings 
        }
    }
}
void sim7000G_init (void){
    ESP_LOGI(TAG_SIM,"UART for Sim7000G has been initialized"); 
    uart_config_t uart_config = {
    .baud_rate =115200, 
    .data_bits = UART_DATA_8_BITS, 
    .parity = UART_PARITY_DISABLE, 
    .stop_bits = UART_STOP_BITS_1, 
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB, 
    }; 

    uart_param_config(UART_PORT_SIM, &uart_config); 
    uart_set_pin(UART_PORT_SIM, UART_SIM_TX, UART_SIM_RX, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE); 
    uart_driver_install(UART_PORT_SIM, 1024, 0, 0, NULL,0);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TSIM_PWR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    //copied from the ino file used for initial testing of the board 
    gpio_set_level(TSIM_PWR,1); 
    ESP_LOGI(TAG_SIM, "TSIM power pin set to low for 1 sec"); 
    //vTaskDelay(1000/portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(TSIM_PWR, 0);
    ESP_LOGI(TAG_SIM, "waiting to boot..."); 
    vTaskDelay(pdMS_TO_TICKS(5000)); //wait to boot 
}

static void send_from_sim(void *arg){
    // char* num_at ="AT+CMGS=\"09555425524\"\r\n";
    while (1){
        if (xSemaphoreTake(xSendGSM, portMAX_DELAY) && uart_at_buffer[0] != '\0'){
            //call at+msg; 
            sendATCommand("AT+CMGS=\"09555425524\"\r\n", ">", 1000); 
            vTaskDelay(500); 
           
            //for duebugging only 
            //ESP_LOGI(TAG_SIM, "%s", uart_at_buffer); 
            
            uart_write_bytes(UART_PORT_SIM, uart_at_buffer, TXT_LEN);
            ESP_LOGI(TAG_SIM,"Send from sim"); 
        }
    }

}
static void module_init(void *arg){
    sim7000G_init(); 
    uint8_t counter = 0; 
    while (1){
        at_success = sendATCommand("AT\r\n", "OK", 500);

            if (at_success)
            {
                ESP_LOGI(TAG_SIM, "Module Initialization done successfully with final count: %d", counter++);
                sendATCommand("AT+CGREG?\r\n","+CGREG: 0,1",500);
                sendATCommand("AT+CMNB=1\r\n","OK",500); 
                sendATCommand("AT+CMGF=1\r\n","OK",500); 
                sendATCommand("AT+CSCS=\"GSM\"\r\n","OK",500); 
                
                xTaskCreate(send_from_sim, "Send-TXT",2048, NULL,9, NULL); 
               
                vTaskDelete(NULL); 
            } 

            else
            {
                ESP_LOGW(TAG_SIM,"Module Initialization fail, with count:%d\n ",counter++);
            }
    }
}





bool sendATCommand(char* command, char* expectedResponse, int timeoutMs) // Sending AT Command
{
    uint8_t buffer[UART_SIM_BUFFER];

    memset(buffer , 0 ,UART_SIM_BUFFER);

    uart_write_bytes(UART_PORT_SIM, command, strlen(command));  

    vTaskDelay(pdMS_TO_TICKS(100)); 

    ESP_LOGI(TAG_SIM ,"Write done");

    bool responseReceived = parse_data(buffer, UART_SIM_BUFFER , timeoutMs, expectedResponse);

    if (responseReceived) 
    {
        ESP_LOGI(TAG_SIM, "Command sent successfully!");

        return true;

    } else 
    {
        ESP_LOGW(TAG_SIM, "Failed to receive expected response!");
        return false;
    }
}


void set_text_message (uint8_t count){
    float temp= 0; 
    float ph = 0; 
    float DO = 0; 
    float ave_temp = 0 ; 
    float ave_do = 0; 
    float ave_ph = 0;
    char timestamp[25];
    char temp_status[20]; 
    char ph_status[20]; 
    char do_status[20]; 

    memset(timestamp, '\0', sizeof(timestamp)); 
    memset(temp_status, '\0', sizeof(temp_status)); 
    memset(do_status, '\0', sizeof(do_status));
    memset(ph_status, '\0', sizeof(ph_status)); 

    strncpy(timestamp, sensor_recent[count].timestamp, sizeof(timestamp));
    for (int i=1; i <= count;i++){
        temp += sensor_recent[i].temp; 
        ESP_LOGI(TAG_SENSOR, "temp:%0.2f", temp); 
        DO += sensor_recent[i].DO; 
        ph += sensor_recent[i].pH; 
    }
    ave_temp = temp/count; 
    ave_do = DO/count; 
    ave_ph = ph/count; 
    /*
    Suma ng mga datos para sa inyong aquaculture site sa ganap na 2025-07-29T9:03:00.


    Temperatura: 
    pH:
    Dissolved Oxygen: 
    Batterya ng Sensor Node:

    
    <Wala pang interpretasyon, might involve nuance in the coding and the meaning of the parameters> 
    
    if (ave_temp >= 25 && ave_temp <= 31){
        snprintf(temp_status, sizeof(temp_status), "Normal temp at %0.2f", ave_temp);
        ESP_LOGI(TAG_SIM, "%s",temp_status); 
    } 
    if (ave_ph >=6.5 && ave_ph <= 8.5){
        snprintf(ph_status, sizeof(ph_status), "Normal ph at %0.2f", ave_ph);
        ESP_LOGI(TAG_SIM, "%s",ph_status); 
    }
    if (ave_do >= 5){
        snprintf(do_status, sizeof(do_status), "Normal do at %0.2f", ave_do);
        ESP_LOGI(TAG_SIM, "%s",do_status);
        //print normal 
    }*/

    if (ave_temp >= 25 && ave_temp <= 31){
        snprintf(temp_status, sizeof(temp_status), "%0.2f (Normal)", ave_temp);
    }  else if (ave_temp < 25){
        snprintf(temp_status, sizeof(temp_status),"%0.2f (Mababa)",ave_temp);

    } else if(ave_temp < 25){
        snprintf(temp_status, sizeof(temp_status),"%0.2f (Mataas)",ave_temp);
    }
    ESP_LOGI(TAG_SIM, "%s",temp_status); 

    if (ave_ph >=6.5 && ave_ph <= 8.5){
        snprintf(ph_status, sizeof(ph_status), "%0.2f (Normal)", ave_ph);
        ESP_LOGI(TAG_SIM, "%s",ph_status); 
    } else if (ave_ph < 6.5){
        snprintf(ph_status, sizeof(ph_status),"%0.2f (Mababa)",ave_ph);
    } else if (ave_ph >8.5){
        snprintf(ph_status, sizeof(ph_status),"%0.2f (Mataas)",ave_ph);
    }
    ESP_LOGI(TAG_SIM, "%s",ph_status);

    if (ave_do >= 5){
        snprintf(do_status, sizeof(do_status), "%0.2f (Mataas)", ave_do);
        
    }else{
        snprintf(do_status, sizeof(do_status), "%0.2f (Mababa)", ave_do);
    }
    ESP_LOGI(TAG_SIM, "%s",do_status);


    snprintf(uart_at_buffer, TXT_LEN,
    "Suma ng mga datos para sa inyong aquaculture site sa ganap na %s.\n\n Temperatura: %s\n pH: %s\n DO: %s\x1A",
    timestamp, temp_status, ph_status, do_status);   
    ESP_LOGI(TAG_SIM, "%s",uart_at_buffer); 
    // ESP_LOGI(TAG_SIM, "TEMP: %0.2f, DO:%0.2f, pH:%0.2f, timestamp:%s", ave_temp, ave_do, ave_ph, timestamp); 

}

bool parse_data(uint8_t* data, size_t len, int timeout_ms , const char* word) // function for pasing data 
{
    char resp[UART_SIM_BUFFER] = {0};   // Allocate a buffer to hold the received data

    TickType_t start_time = xTaskGetTickCount(); // get the start time of the loop
    

    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS))
    {
        int bytes_read = uart_read_bytes(UART_PORT_SIM, data, len, pdMS_TO_TICKS(500));

        if (bytes_read > 0)
        {
            // Append the received data to the buffer
            strncat(resp, (const char*)data, bytes_read); 
        

            // Check if the word is present in the buffer
            if (strstr(resp, word) != NULL)
            {
                return true;
            }
        }
    }
    
    return false;
}
void copy_buffer (Sensor_Data* original, Sensor_Data* copy){
    for(int i=1; i <= SENSOR_MAX_IND  ; i++){
        copy[i]= original[i]; //store data from last transmit 
    }
}