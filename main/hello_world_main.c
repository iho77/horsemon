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
#include "esp_system.h"
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
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "string.h"
#include "lora.h"




#define DEFAULT_VREF    1109        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   512   //ULP got 512 readings i.e.
#define FREQ 32 // sampling frequency
#define DATA_BUF_SIZE 512 //size of buffer for acc readings
#define FEATURES_NUM 2
#define DEV_ADDR 0x01
#define NUM_OF_WINDOWS 4 //number of winodws size = NO_OF_SAMPLES to store before calculate features

#define STORAGE_NAMESPACE "storage"


extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");



// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
const char *base_path = "/spiflash";

static esp_adc_cal_characteristics_t adc_chars;

float data[7][NO_OF_SAMPLES*NUM_OF_WINDOWS];
uint16_t window[3][NO_OF_SAMPLES],batch[3][NO_OF_SAMPLES*NUM_OF_WINDOWS];
float features[FEATURES_NUM];



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


int32_t restart_counter;

typedef enum {
	READ = 0,
	SAVE = 1,
	RESET = 2
} opcode_t;



void get_window(){
	        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, (esp_adc_cal_characteristics_t *)&adc_chars);
			for (int i=0;i<NO_OF_SAMPLES;i++){
			window[0][i] = esp_adc_cal_raw_to_voltage((&ulp_channel_0_measurements)[i] & UINT16_MAX, &adc_chars);
			window[1][i] = esp_adc_cal_raw_to_voltage((&ulp_channel_1_measurements)[i] & UINT16_MAX, &adc_chars);
			window[2][i] = esp_adc_cal_raw_to_voltage((&ulp_channel_2_measurements)[i] & UINT16_MAX, &adc_chars);
			//printf("%d;%d;%d\n",window[0][i],window[1][i],window[2][i]);

		}
}


void proceed_butch(){
	for (int i=0;i<NO_OF_SAMPLES*NUM_OF_WINDOWS;i++){
		data[0][i] = batch[0][i];
		data[1][i] = batch[1][i];
		data[2][i] = batch[2][i];
		data[3][i] = (float)sqrt(pow(data[0][i],2)+pow(data[1][i],2)+pow(data[2][i],2));
		data[4][i] = 0.0; //roll
		data[5][i] = 0.0; //pitch
		data[6][i] = 0.0; //yow
		printf("%f;%f;%f;%f\n",data[0][i],data[1][i],data[2][i],data[3][i]);
	}
}


static void init_ulp_program();
static void start_ulp_program();
esp_err_t restart_counter_op(opcode_t), data_op(opcode_t);


void proceed_features(){
	for(int i=0;i<FEATURES_NUM;i++) features[i] = features_calc[i]();
}

void app_main()
{   static const char *TAG = "main";

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    esp_err_t err = nvs_flash_init();

       if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
           // NVS partition was truncated and needs to be erased
           // Retry nvs_flash_init
           ESP_ERROR_CHECK(nvs_flash_erase());
           err = nvs_flash_init();
       }
       ESP_ERROR_CHECK( err );


    if (cause != ESP_SLEEP_WAKEUP_ULP) {
	        ESP_LOGI(TAG,"Not ULP wakeup");
    	    restart_counter_op(RESET);
            init_ulp_program();
	    } else {
	        ESP_LOGI(TAG,"Deep sleep wakeup");
	    	restart_counter_op(READ);
	    	ESP_LOGI(TAG,"Restart counter:%d",restart_counter);


	    	if (restart_counter == NUM_OF_WINDOWS) {
	    		ESP_LOGI(TAG, "Collected %d windows", restart_counter);
	    		restart_counter_op(RESET); //reset_counter = 0
                data_op(READ); //read 4 windows to data[]
                data_op(RESET); //erase data
                proceed_butch();
                proceed_features(); //made features calculations
                /*
               	lora_init();
               	lora_set_frequency(433e6);
               	lora_enable_crc();
                lorasend(); //send features to gw */
           	} else {
           		get_window();
           		data_op(SAVE);
           		restart_counter_op(SAVE);
           		ESP_LOGI(TAG, "Save restart counter:%d",restart_counter);

           	}



	    }

   	    ESP_LOGI(TAG, "Entering deep sleep");
        fflush(stdout);
	    start_ulp_program();
	    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
	    //seems it does not have any sense on Heltec esp32-LoRa module
	    rtc_gpio_isolate(GPIO_NUM_4);
	    rtc_gpio_isolate(GPIO_NUM_15);
	    rtc_gpio_isolate(GPIO_NUM_25);
	    esp_deep_sleep_start();



}

esp_err_t restart_counter_op(opcode_t cmd){
	nvs_handle my_handle;
	esp_err_t err;

	// Open
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) return err;

	// Read
	switch(cmd){
	case READ:
		 restart_counter = 0; // value will default to 0, if not set yet in NVS
	     err = nvs_get_i32(my_handle, "restart_conter", &restart_counter);
	 	 if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
	 	 break;

	case SAVE:
		restart_counter++;
	    err = nvs_set_i32(my_handle, "restart_conter", restart_counter);
	    if (err != ESP_OK) return err;
	    break;

	case RESET:
		err = nvs_set_i32(my_handle, "restart_conter", 0);
        if (err != ESP_OK) return err;
        break;
	}
	    // Commit written value.
	    // After setting any values, nvs_commit() must be called to ensure changes are written
	    // to flash storage. Implementations may write to storage at other times,
	    // but this is not guaranteed.
     	err = nvs_commit(my_handle);
	    if (err != ESP_OK) return err;

	    // Close
	    nvs_close(my_handle);
	    return ESP_OK;
}



esp_err_t data_op(opcode_t cmd){

	static const char *TAG = "data_op";

	ESP_LOGI(TAG, "Mounting FAT filesystem");
	FILE *f;
	// To mount device we need name of device partition, define base_path
	// and allow format partition in case if it is new one and was not formated before
	const esp_vfs_fat_mount_config_t mount_config = {
	            .max_files = 4,
	            .format_if_mount_failed = true,
	            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
	    };
	esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
	if (err != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
	        return err;
	   }

	switch(cmd){
	case SAVE:
		f = fopen("/spiflash/data.bin", "ab");
	    if (f == NULL) {
		  ESP_LOGE(TAG, "Failed to open file for writing");
		  return ESP_FAIL;}
	    fwrite(window,sizeof(window),1,f);
	    fclose(f);
	    ESP_LOGI(TAG, "File written");
	    break;
	case READ:
		f = fopen("/spiflash/data.bin", "rb");
		if (f == NULL) {
		  ESP_LOGE(TAG, "Failed to open file for writing");
		return ESP_FAIL;}
		fread(batch,sizeof(batch),1,f);
		fclose(f);
		ESP_LOGI(TAG, "File read");
		break;
	case RESET:
		f = fopen("/spiflash/data.bin", "w");
		if (f == NULL) {
		  ESP_LOGE(TAG, "Failed to open file for writing");
		 return ESP_FAIL;}
		fclose(f);
		ESP_LOGI(TAG, "File erased");
		break;

	}

	 ESP_LOGI(TAG, "Unmounting FAT filesystem");
	 ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));
	 ESP_LOGI(TAG, "Done");
	 return ESP_OK;
}


void lorasend(){
	uint8_t msg[6+1+FEATURES_NUM*sizeof(float)]; //features + addr + number of features
	//printf("Size of LoRa packet is:%d\n", sizeof(msg));
	esp_efuse_mac_get_default(msg);
	//printf("Device Address is:%02x:%02x:%02x:%02x:%02x:%02x\n",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5]);
	msg[6]=FEATURES_NUM;
	memcpy(&msg[7],features,sizeof(features));
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
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_6);
    adc1_ulp_enable();



    /* Set ULP wake up period to 32Hz */
    ulp_set_wakeup_period(0, 31250);

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

