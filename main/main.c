#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "stdlib.h"
#include "freertos/queue.h"

//***************************************************************DEFINICION DE VARIABLES***************************************
//Definiciones del UART
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024 * 2
#define TASK_MEMORY 1024 *2
#define PIN_TX 1
#define PIN_RX 3

#define lenght_Rx 13

//Definicion de ISR (botones)
#define button_SUM  22
#define button_RES 23

// Definimos los pines de los LEDS
#define ledBlink        2
#define ledR            33
#define ledG            25
#define ledB            26


//Definimos comandos ON Y OFF de los LEDs
#define LEDR_ON         gpio_set_level(ledR,1);
#define LEDG_ON         gpio_set_level(ledG,1);
#define LEDB_ON         gpio_set_level(ledB,1);
#define LED_BLINK_ON    gpio_set_level(ledBlink,1);

#define LEDR_OFF         gpio_set_level(ledR,0);
#define LEDG_OFF         gpio_set_level(ledG,0);
#define LEDB_OFF         gpio_set_level(ledB,0);
#define LED_BLINK_OFF    gpio_set_level(ledBlink,0);



//Definimos el periodo del Blink
#define CONFIG_BLINK_PERIOD 1000


static const char *tag = "Main";





//***************************************************************PROTOTIPADO DE VARIABLES***************************************
//Variables del UART
static QueueHandle_t uart_queue;
uint8_t transmit_text[200];

uint8_t len;
char message_to_send[100];

uint8_t pos_in;
uint8_t pos_fin;

uint32_t ls_red;
uint32_t li_red;
uint32_t ls_green;
uint32_t li_green;
uint32_t ls_blue;
uint32_t li_blue;

float threshold_temp = 3.3455;

//Variables del TIMER
TimerHandle_t xTimers;
int interval = 10;
int timerId = 1;
uint32_t count_timer = 0;
uint32_t count_blink = 0;


//Variables de INTERRUPCIONES
int banderaSUM = 0;
int banderaRES = 0;
int threshold_freq = 1;

//Variables del ADC
int raw = 0;
float temp = 1.0;




//***************************************************************PROTOTIPADO DE FUNCIONES***************************************
//Prototipado de funciones del TIMER
void vTimerCallback(TimerHandle_t pxTimer);


//Prototipado de funciones de los LEDs
esp_err_t init_led(void);


//Prototipado de funciones del UART
static int Get_number(char *data);
static void Led_RGB_Limits_Temp (int temp_li_green ,int temp_li_blue, int temp_li_red, int temp_ls_green ,int temp_ls_blue, int temp_ls_red);
static void print_Limits(void);
static void print_Threshold(void);


//Prototipado de funciones de INTERRUPCIONES
void isr_handlerSUM(void *args);
void isr_handlerRES(void *args);


//***************************************************************INICIALIZACIÓN de PERIFÉRICOS***************************************
// Función de inicialización del TIMER (Cada 10 ms entra a la interrupción)
esp_err_t init_timer(void)
{
    ESP_LOGI(tag, "Timer iniciando timer.");
    xTimers = xTimerCreate("Timer",                   // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(interval)), // The timer period in ticks.
                           pdTRUE,                    // The timers will auto-reload themselves when they expire.
                           (void *)timerId,           // Assign each timer a unique id equal to its array index.
                           vTimerCallback             // Each timer calls the same callback when it expires.
    );
    if (xTimers == NULL)
    {
        // The timer was not created.
        ESP_LOGI(tag, "Timer no creado.");
    }
    else
    {
        // Start the timer.  No block time is specified, and even if one was
        // it would be ignored because the scheduler has not yet been
        // started.
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            ESP_LOGI(tag, "The timer could not be set into the Active state");
        }
    }
    return ESP_OK;
}

// Función de inicialización del ADC
esp_err_t init_ADC(void){
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //GPIO Vp
    adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}

// Función de inicialización del UART
static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM,BUF_SIZE,BUF_SIZE,20,&uart_queue,0);
    //xTaskCreate(uart_task, "uart_event_task", TASK_MEMORY,NULL,12,NULL);
}

// Función de inicialización de INTERRUPCIONES
esp_err_t init_isr(void)
{
    gpio_config_t pISRConfig;
    pISRConfig.pin_bit_mask = ((1ULL << button_SUM)|(1ULL << button_RES));
    pISRConfig.mode = GPIO_MODE_DEF_INPUT;
    pISRConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pISRConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pISRConfig.intr_type = GPIO_INTR_NEGEDGE;
    
    gpio_config(&pISRConfig); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(button_SUM,isr_handlerSUM,NULL);
    gpio_isr_handler_add(button_RES,isr_handlerRES,NULL);
    return ESP_OK;
}

// Función de inicialización de los pines
esp_err_t init_led(void) 
{
    // Inicializo el pin del led ROJO
    gpio_reset_pin(ledR);
    gpio_set_direction(ledR, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led VERDE
    gpio_reset_pin(ledG);
    gpio_set_direction(ledG, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led AZUL
    gpio_reset_pin(ledB);
    gpio_set_direction(ledB, GPIO_MODE_OUTPUT);


    // Inicializo el pin del led AZUL
    gpio_reset_pin(ledBlink);
    gpio_set_direction(ledBlink, GPIO_MODE_OUTPUT);

    return ESP_OK;
}


void app_main(void)
{
    init_timer();
    init_led();
    init_isr();
    init_ADC();
    init_uart();

    const char* data = (const char *)malloc(BUF_SIZE);
    uart_event_t event;

    while(1)
    {
        /***********************************************PROGRAMACIÓN OBTENER TEMPERATURA DE LA NTC (ADC)******************************************************/
        raw = adc1_get_raw(ADC1_CHANNEL_0);

        /***********************************************PROGRAMACIÓN FRECUENCIA DEL BLINK (BOTONES)**********************************************************/
        if (banderaSUM == 1 ){
            if (threshold_freq < 10 ){ //Nos aseguramos que no se desborde
                threshold_freq ++; //Aumentamos la frecuencia del BLINK
            }
            banderaSUM = 0; //Reiniciamos la bandera
        }

        if (banderaRES == 1 ){
            if (threshold_freq > 1 ){ //Nos aseguramos que no se desborde
                threshold_freq --; //Disminuimmos la frecuencia del BLINK
            }
            banderaRES = 0; //Reiniciamos la bandera
        }

        /*************************************************************PROGRAMACIÓN UART**********************************************************************/
        if (xQueueReceive(uart_queue,(void *)&event,pdMS_TO_TICKS(100))){ //Si recibió un evento de UART
            bzero( data , BUF_SIZE);
            switch (event.type){
            case UART_DATA: //Si recibió un mensaje
                uart_read_bytes(UART_NUM, data, event.size,pdMS_TO_TICKS(100));
                //uart_write_bytes(UART_NUM,(const char *)data,event.size);

                if ( strstr(data,"LEDR_MAX") != 0){
			        ls_red = Get_number(data);
			        //ban_recibido_msj = 0;
		        }

                if (strstr(data,"LEDR_MIN") != 0){
                    li_red = Get_number(data);
		        }

                if ( strstr(data,"LEDG_MAX") != 0){
			        ls_green = Get_number(data);
			        //ban_recibido_msj = 0;
		        }

                if (strstr(data,"LEDG_MIN") != 0){
                    li_green = Get_number(data);
		        }

                if ( strstr(data,"LEDB_MAX") != 0){
			        ls_blue = Get_number(data);
		        }

                if (strstr(data,"LEDB_MIN") != 0){
                    li_blue = Get_number(data);
		        }

                if(strstr(data,"THRESHOLD") != 0){
                    threshold_temp = Get_number(data);
                }

                if (strstr(data,"GET_LIMITS") != 0){
                    print_Limits();                  
		        }

                if(strstr(data,"GET_THRESHOLD") != 0){
                    print_Threshold();
                }

                uart_flush(UART_NUM); //Asegura que todo el DATA está transmitido antes de que pase a la siguiente línea
                break;
            default:
                break;
            }
        }

        /*********************************Control encendido LED RGB***************************************/
	    Led_RGB_Limits_Temp(li_green ,li_blue, li_red,ls_green ,ls_blue, ls_red );
    }

}
//***********************************************************************FUNCIONES DEL UART**********************************************************

/****************************************Función para obtener el número del UART***************/
static int Get_number(char *data)
{
	uint32_t result;

	for (int i = 0; i<lenght_Rx + 1; i++){
		if (data[i] == '$'){
			pos_in = i;
			for (int j = i+1; j<lenght_Rx+1; j++){
				if (data[j] == '$'){
					pos_fin = j;
					break;
				}
			}
			break;
		}
	}


	result = 0;
	for (int z = pos_in+1; z<=pos_fin-1; z++ ){ 
		result += (data[z]-48)*(pow(10, pos_fin-1-z));
	}
    printf("%d\n",result);

	return result;

}


/*************************************Función para Control encendido LED RGB*******************************/
static void Led_RGB_Limits_Temp (int temp_li_green ,int temp_li_blue, int temp_li_red, int temp_ls_green ,int temp_ls_blue, int temp_ls_red){

	LEDR_OFF;
	LEDG_OFF;
	LEDB_OFF;

	if ((10>temp_li_green) && (19 <= temp_ls_green)){
		LEDG_ON;
	}

	if ((21>temp_li_blue) && (29 <= temp_ls_blue)){
		LEDB_ON;
	}

	if ((31>temp_li_red) && (39 <= temp_ls_red)){
		LEDR_ON;
	}
}

/****************************************Función para IMPRIMIR los límites***************/
static void print_Limits(void){
    len = sprintf((char *)&message_to_send[0], "ROJO->  LI: %d  LS: %d \r\n",li_red,ls_red);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
    len = sprintf((char *)&message_to_send[0], "VERDE-> LI: %d  LS: %d \r\n",li_green,ls_green);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
    len = sprintf((char *)&message_to_send[0], "AZUL->  LI: %d  LS: %d \r\n",li_blue,ls_blue);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
}

/****************************************Función para IMPRIMIR el THRESHOLD***************/
static void print_Threshold(void){
    len = sprintf((char *)&message_to_send[0], "El THRESHOLD es: %.2f \r\n",threshold_temp);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
}





//***********************************************************************FUNCIONES DEL TIMER**********************************************************/
void vTimerCallback(TimerHandle_t pxTimer)
{
    count_timer ++;

    if (count_timer > 1000000-1 ){ //Para asegurarnos que trabaje hasta 1 Msegundo -> Evitar desbordamiento
        count_timer = 0;
    }

    if (temp > threshold_temp)
    {
        switch (threshold_freq)
        {
            case 1:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 2:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 3:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 4:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 5:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 6:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 7:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 8:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 9:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            case 10:
            {
                if ( (count_timer % (100/threshold_freq)) == 0 )
                {
                    count_blink++;

                    if (count_blink > 10000-1)
                    {
                        count_blink = 0;
                    }
                }

                if (count_blink % 2 == 0){
                    LED_BLINK_OFF;
                }
                lse{
                    LED_BLINK_ON;
                }
            break;
            }

            default:
            break;
        }
    }
}

//***********************************************************************FUNCIONES DE INTERRUPCIONES**********************************************************/
/*Función para incrementar la frecuencia del BLINK*/
void isr_handlerSUM(void *args)
{
    banderaSUM = 1;
}


/*Función para decrementar la frecuencia del BLINK*/
void isr_handlerRES(void *args)
{
    banderaRES = 1;
}