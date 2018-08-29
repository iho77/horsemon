/* ADC1 Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/rtc_io.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "string.h"
#include "lora.h"




#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64   //Multi sampling
#define DATA_BUF_SIZE 512 //size of buffer for acc readings
#define FEATURES_NUM 2

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


float data[7][DATA_BUF_SIZE];
float features[FEATURES_NUM];

static esp_adc_cal_characteristics_t adc_chars;

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program();

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program();


void lorasend();


float f_mean(){
	return data[0][0];
}

float f_sum(){
	float res = 0.0;
	for (int i=0;i<DATA_BUF_SIZE;i++){
		res += data[0][i];
	}
	return res;
}


float (* features_calc[FEATURES_NUM])() = {f_mean,f_sum};





void proceed_buffer(){


    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, (esp_adc_cal_characteristics_t *)&adc_chars);

	for (int i=0;i<(ulp_sample_counter & UINT16_MAX);i++){
		data[0][i] = (int)(esp_adc_cal_raw_to_voltage((&ulp_channel_0_measurements)[i] & UINT16_MAX, &adc_chars)-1500)/300.0;
		data[1][i] = (int)(esp_adc_cal_raw_to_voltage((&ulp_channel_1_measurements)[i] & UINT16_MAX, &adc_chars)-1500)/300.0;
		data[2][i] = (int)(esp_adc_cal_raw_to_voltage((&ulp_channel_2_measurements)[i] & UINT16_MAX, &adc_chars)-1500)/300.0;
		data[3][i] = (float)sqrt(pow(data[0][i],2)+pow(data[1][i],2)+pow(data[2][i],2));
		data[4][i] = 0.0; //roll
		data[5][i] = 0.0; //pitch
		data[6][i] = 0.0; //yow
		printf("%f;%f;%f;%f\n",data[0][i],data[1][i],data[2][i],data[3][i]);
	}
}

static void init_ulp_program();
static void start_ulp_program();

void app_main()
{

	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

	lora_init();
	lora_set_frequency(433e6);
	lora_enable_crc();

	/*while(1){
		lorasend();
		vTaskDelay(2000/portTICK_PERIOD_MS);
	}*/

    if (cause != ESP_SLEEP_WAKEUP_ULP) {
	  //      printf("Not ULP wakeup\n");

            init_ulp_program();
	    } else {
	  //    printf("Deep sleep wakeup\n");
	        proceed_buffer();
	        for(int i=0;i<FEATURES_NUM;i++){
	        			features[i] = features_calc[i]();
	        		}
	        lorasend();

	    }
	    //printf("Entering deep sleep\n\n");
	    start_ulp_program();
	    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
	    //seems it does not have any sense on Heltec esp32-LoRa module
	    rtc_gpio_isolate(GPIO_NUM_4);
	    rtc_gpio_isolate(GPIO_NUM_15);
	    rtc_gpio_isolate(GPIO_NUM_25);
	    esp_deep_sleep_start();



}

void lorasend(){
	char msg[255];
	msg[0]=1;
	int index = 1;
    for(int i=0;i<FEATURES_NUM;i++){
    	index += sprintf(&msg[index], "%f:", features[i]);
    }
    printf("%s\n",msg);
	lora_send_packet((uint8_t *)(&msg), sizeof(msg));

}


static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Configure ADC channel */
    /* Note: when changing channel here, also change 'adc_channel' constant
       in adc.S */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);
    adc1_ulp_enable();



    /* Set ULP wake up period to 100ms */
    ulp_set_wakeup_period(0, 100000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO15 may be connected to ground to suppress boot messages.
     * GPIO12 may be pulled high to select flash voltage.
     */

}

static void start_ulp_program()
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
}

