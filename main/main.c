#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


//defines
#define UART_HELTEC_TX (33)
#define UART_HELTEC_RX (34)
#define UART_SIM_TX (26) 
#define UART_SIM_RX (27)//based on its pin def, will not be entirely used

#define UART_BUFFER_SIZE 128
#define UART_PORT_H (UART_NUM_1) //uart port fo the heltec
#define UART_PORT_SIM (UART_NUM_2) //uart port for the sim card module

#define SENSOR_MAX_IND (5)

// private variables
char* tx_data = "Hello world from TSIM\r\n"; 
char rx_data[128];

uint8_t new_line_ctr=0;
char uart_rx_buffer[UART_BUFFER_SIZE];  //for the data sent copied from the rx_buffer
uint16_t uart_rx_index = 0;
char uart_rx_char;
bool parse_ready = false; //gawing binary semaphore (kapag gumagana na) 

static const char *TAG_UART = "UART_HELTEC_LOG";
static const char *TAG_SENSOR = "PARSE_LOG"; 
static const char *TAG_SIM = "UART_SIM_LOG";


typedef struct {
    uint16_t id;
    uint8_t type;
    //include timestamp 
    float temp;
    float DO;
    float pH;
    float batt;
    bool error; 
} Sensor_Data; 

Sensor_Data sensor_received;  //for debugginug only 
Sensor_Data sensor_buffer[SENSOR_MAX_IND]; 
bool sensor_low_batt = false; 

//prototype functions 
void init_uart (uart_port_t uart_num, int tx, int rx, int baud); 
static void uart_event_task (void* arg); 
static void process_uart_task(void* arg); 
static void parse_sensor_data (char* buffer, Sensor_Data* sensor); 
static void send_to_sim(void *arg); 


// rtos handles
static QueueHandle_t uart_queue;
static QueueHandle_t data_queue;

static TaskHandle_t xProcess; 
static BaseType_t process_task; 

static TaskHandle_t xSim; 
static BaseType_t sim_task; 

static SemaphoreHandle_t xParseReady;

//---------------------------------------------------------------------------------//
//start of the main code 

void app_main(void)
{
    init_uart(UART_PORT_H, UART_HELTEC_TX, UART_HELTEC_RX,38400); 
    //init_uart(UART_PORT_SIM, UART_SIM_TX, UART_SIM_RX); 

    xParseReady = xSemaphoreCreateBinary(); 
    data_queue = xQueueCreate(5,sizeof(Sensor_Data)); 

    xTaskCreate(uart_event_task, "uart_event", 4096, NULL, 5, NULL); 
    process_task = xTaskCreate(process_uart_task, "uart_process", 4096, NULL, 6, &xProcess);
    configASSERT(process_task == pdPASS); 
    
    sim_task = xTaskCreate(send_to_sim, "Sim Task", 2048, NULL, 7, &xSim); 
    configASSERT(sim_task == pdPASS); 


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
                        }
                    } else if (uart_rx_index < UART_BUFFER_SIZE - 1) {
                        uart_rx_buffer[uart_rx_index++] = data[i];
                    }
                }
                break;
            default:
                break;
            }

            /*read the 11 byte of the payload

            if  (uart_rx_buffe[11] == 4 ){
                //read rx buffer at awc, dahil isang digit lang naman yun 
             
            } else if (uart_rx_buffe[11] == 0 ){
                parse_payload(void);
             }    


            */  
            
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
    // char print[10]; 
    char*tx_ack ="type:2,ACK"; 
    char rx_data_buffer[UART_BUFFER_SIZE];

    while (1){
        if (xSemaphoreTake(xParseReady, portMAX_DELAY)){ 
            memset(rx_data_buffer,'\0',UART_BUFFER_SIZE); 
            strcpy(rx_data_buffer, &uart_rx_buffer[5]);
            memset(uart_rx_buffer,'\0',UART_BUFFER_SIZE); 
            switch (rx_data_buffer[5]){
                case '0':
                //parsepayload
                    parse_sensor_data(rx_data_buffer,&sensor_received);
                   // xQueueSend(data_queue,&sensor_received, portMAX_DELAY);
                    ESP_LOGI(TAG_SENSOR, "Data sent to queue"); 
                  
                    break;
                case '4': 
                //trigger ack; 
                    vTaskDelay(5000/portTICK_PERIOD_MS); 
                    uart_write_bytes(UART_PORT_H,tx_ack,10); 
                    ESP_LOGI(TAG_UART, "Transmitted to Heltec."); 
                    break; 
                
                default:
                    ESP_LOGW(TAG_UART, "Invalid payload format."); 
                    break; 
                
            }
            
        }
    }
}
static void parse_sensor_data (char* buffer, Sensor_Data* sensor){

    if (buffer == NULL || sensor == NULL) {
        ESP_LOGE(TAG_SENSOR, "Null pointer passed to parse_sensor_data");
        return;
    }

    sensor->error = false; 

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
                    } else{
                        sensor->error = true; //low batt transmit
                        }

				}
		token = strtok(NULL, ",");

    }        
    ESP_LOGI(TAG_SENSOR, "Parsed sensor data: ID=%d, Temp=%.2f, pH=%.2f, DO=%.2f, Batt=%.2f",sensor->id, sensor->temp, sensor->pH, sensor->DO, sensor->batt);

}


static void send_to_sim(void *arg){
    while (1){
        
    }
}