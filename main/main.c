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
#define UART_SIM_TX (26) 
#define UART_SIM_RX (27)//based on its pin def, will not be entirely used

#define UART_BUFFER_SIZE 128
#define UART_PORT_H (UART_NUM_1) //uart port fo the heltec
#define UART_PORT_SIM (UART_NUM_2) //uart port for the sim card module

#define SENSOR_MAX_IND (3)
#define SENSOR_MIN_IND (1)
#define SENSOR_RECEIVE_TIMEOUT (60000)
#define timestamp_len  (25)
#define TXT_LEN (160)

// rtos handles
static QueueHandle_t uart_queue;
static QueueHandle_t data_queue; 

static TaskHandle_t xProcess; 
static BaseType_t process_task; 

static TaskHandle_t xProtoSim; 
static BaseType_t pro_to_sim;

//static TaskHandle_t xSimInit; 
//static BaseType_t sim_init;   

static SemaphoreHandle_t xParseReady;
static SemaphoreHandle_t xSensorReady;
static TimerHandle_t xReceiveTimeout = NULL;


// private variables

char* tx_data = "Hello world from TSIM\r\n"; 
char rx_data[128];

uint8_t new_line_ctr=0;
char uart_rx_buffer[UART_BUFFER_SIZE];  //for the data sent copied from the rx_buffer
uint16_t uart_rx_index = 0;
char uart_rx_char;
bool parse_ready = false; //gawing binary semaphore (kapag gumagana na) 

static const char *TAG_UART = "UART_HELTEC_LOG";
static const char *TAG_SENSOR = "SENSOR_LOG"; 
static const char *TAG_SIM = "UART_SIM_LOG";

static bool sensor_timeout = false; //for the payload timer
static bool sensor_ready = false; 
static bool reTx = false; 

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
    uint8_t curr_index;     
} Sensor_Data; 


Sensor_Data sensor_buffer[SENSOR_MAX_IND+1];
bool index_received[SENSOR_MAX_IND+1] ={false};
uint8_t received_count= 0;

//prototype functions 
void init_uart (uart_port_t uart_num, int tx, int rx, int baud); 
static void uart_event_task (void* arg); 
static void process_uart_task(void* arg); 
static void parse_sensor_data (char* buffer, Sensor_Data* sensor); 
static void process_to_sim(void *arg); 
void set_text_message (Sensor_Data* data_buffer, char* to_send, uint8_t count); 
static void send_from_sim(void* arg); 
void sensor_timeout_callback(TimerHandle_t xTimer); 



//---------------------------------------------------------------------------------//
//start of the main code 

void app_main(void)
{
    init_uart(UART_PORT_H, UART_HELTEC_TX, UART_HELTEC_RX,38400); 
    //init_uart(UART_PORT_SIM, UART_SIM_TX, UART_SIM_RX); 

    xParseReady = xSemaphoreCreateBinary(); 
   // xSensorReady = xSemaphoreCreateBinary();
   //  configASSERT(xReceiveTimeout ==pdPASS); 

     
    data_queue = xQueueCreate(1,(sizeof(Sensor_Data))*(SENSOR_MAX_IND+1)); // queue initialization
    
    xReceiveTimeout = xTimerCreate("Sensor-Timeout", pdMS_TO_TICKS(SENSOR_RECEIVE_TIMEOUT), pdFALSE, NULL, sensor_timeout_callback); 


    xTaskCreate(uart_event_task, "uart_event", 4096, NULL, 5, NULL); 
    process_task = xTaskCreate(process_uart_task, "uart_process", 4096, NULL, 6, &xProcess);
    configASSERT(process_task == pdPASS); 
    
    pro_to_sim = xTaskCreate(process_to_sim, "process_task", 4096, NULL, 7, &xProtoSim); 
    configASSERT(pro_to_sim == pdPASS); 

   // sim_init = xTaskCreate(send_from_sim, "Sim Task",4096, NULL, 4, &xSimInit);
   // configASSERT(sim_init==pdPASS);



}

static void uart_event_task (void* arg){
    uart_event_t event;
    uint8_t data[UART_BUFFER_SIZE]; 
    int new_line_ctr = 0;

    while (true) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
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

void init_uart (uart_port_t uart_num, int tx, int rx, int baud){
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
    uart_driver_install(uart_num, 1024, 0, 20, &uart_queue,0);
    
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
    Sensor_Data sensor_buffer[SENSOR_MAX_IND+1]; 
    char text_message[TXT_LEN]; 
    uint8_t valid_count;
    char*tx_ack ="type:1,ACK"; 
    //char* retrans ="type:3";
    char index [10];
    char temp[3] ;  
  //  Sensor_Data sensor_received; 
    uint8_t ind_buffer[5]; 
 //   uint8_t index_copy = 0; 
    
    while (1){
            
            if (xQueueReceive(data_queue, &sensor_buffer, portMAX_DELAY)){
                valid_count =0 ;    
            //count valid entries ;if not, get the index aand form the retransmit packet 
                for(int i=1; i <= SENSOR_MAX_IND  ; i++){
                        if (sensor_buffer[i].index != 0){
                            //ind_buffer[i] = sensor_buffer[i].index; 
                            valid_count ++;
                        }else{
                            ind_buffer[i]= 1; 
                        }
                     ESP_LOGI(TAG_SIM,"Expected lost indices  in the sensor buffer, %d",sensor_buffer[i].index);
                     ESP_LOGI(TAG_SIM, "index buffer %d", ind_buffer[i]); 
                }
                
                    if (valid_count >= SENSOR_MAX_IND-1){
                        ESP_LOGI(TAG_SIM, "valid count %i", valid_count); 
                        uart_write_bytes(UART_PORT_H,tx_ack,strlen(tx_ack)); //send acknowledgement for valid nmumber of data 
                        //proceed to averaging all 
                        //set_text_message(sensor_buffer,text_message,valid_count); //enter the function with debugging points
                        //semaphore for the text 


                        memset(sensor_recent, 0, sizeof(sensor_recent)); 
                    } else {
                        for(int i=1; i <= SENSOR_MAX_IND  ; i++){
                            sensor_recent[i]= sensor_buffer[i];  
                            if (ind_buffer[i]){
                                memset(temp, '\0', 1);
                                sprintf(temp, "%d,", i); 
                                strcat(index, temp);
                            }
                            
                        }
                        char retransmit [30] = "type:3,ind:";
                        strcat(retransmit,index);
                        ESP_LOGI(TAG_SIM, "sending restrans string %s", retransmit); 
                        uart_write_bytes(UART_PORT_H,retransmit,sizeof(retransmit)); 
                        memset(retransmit, '\0', sizeof(retransmit));
                        memset(index, '\0', sizeof(index)); 
                        memset(ind_buffer,0, sizeof(ind_buffer)); 
                        //valid_count = 0; 
                    }
                memset(sensor_buffer, 0, sizeof(sensor_buffer)); 
               // memset(ind_buffer, 0, sizeof(ind_buffer)); 

 
         
           // memset(ind_buffer, '\0', sizeof(ind_buffer)); 
            //average all readings 
        }
    }
}

void set_text_message (Sensor_Data* data_buffer, char* to_send, uint8_t count){
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

    strncpy(timestamp, data_buffer[count].timestamp, sizeof(timestamp));
    for (int i=1; i <= count;i++){
        temp += data_buffer[i].temp; 
        DO += data_buffer[i].DO; 
        ph += data_buffer[i].pH; 
    }
    ave_temp = temp/count; 
    ave_do = DO/count; 
    ave_ph = ph/count; 
    /*
    Suma ng mga datos para sa inyong aquaculture site nitong date, time.
    
    Temperatura: 
    pH:
    Dissolved Oxygen: 

    Ang temperatura ay <masyadong mababa, nasa tamang 
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

    snprintf(to_send, TXT_LEN, "Time: %s\n%s\n%s\n%s", timestamp, temp_status, ph_status, do_status);
    ESP_LOGI(TAG_SIM, "TEMP: %0.2f, DO:%0.2f, pH:%0.2f, timestamp:%s", ave_temp, ave_do, ave_ph, timestamp); 

}

