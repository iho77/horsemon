/* ADC1 Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <lwip/sockets.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/rtc_io.h"
#include "driver/rtc_cntl.h"
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
#include "config.h"
#include "features.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#define USE_TASK_NOTIFICATIONS 1

#define STORAGE_NAMESPACE "storage"
#define SENDER_PORT_NUM 9999
#define RECEIVER_PORT_NUM 9999
#define RECEIVER_IP_ADDR "192.168.1.2"

char my_ip[32];


extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");



// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

static EventGroupHandle_t wifi_event_group;
SemaphoreHandle_t ulp_isr_sem;

static TaskHandle_t xTaskToNotify = NULL;

// Mount path for the partition
const char *base_path = "/spiflash";

static esp_adc_cal_characteristics_t adc_chars;

const int IPV4_GOTIP_BIT = BIT0;
const int ULP_GOT_WINDOWS = BIT1;

int32_t window[NO_OF_SAMPLES][3],batch[NO_OF_SAMPLES*NUM_OF_WINDOWS][3];

float features[FEATURES_NUM];

int32_t restart_counter;

//int socket;

typedef enum {
	READ = 0,
	SAVE = 1,
	RESET = 2
} opcode_t;

typedef enum {
	LoRa = 0,
	RAW = 1
} op_mode_t;

op_mode_t op_mode = LoRa, check_mode();

esp_err_t restart_counter_op(opcode_t), data_op(opcode_t);


static void start_ulp_program(), init_ulp_program(),start_ulp_program(), lorasend(),get_window(),
		proceed_features(),initialise_wifi(), ulp_isr(void *),udp_sender(void *pvParameters);

void app_main()
{
	static const char *TAG = "main";
	int result;
	uint32_t txpos=0;

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
	        op_mode = check_mode();
	        restart_counter_op(RESET);
    	    data_op(RESET);
            init_ulp_program();
     	    } else {
	        ESP_LOGI(TAG,"Deep sleep wakeup, LoRa mode");
	    	restart_counter_op(READ);
	    	ESP_LOGI(TAG,"Restart counter:%d",restart_counter);
	    	 		if (restart_counter == NUM_OF_WINDOWS) {
	    			    		ESP_LOGI(TAG, "Collected %d windows", restart_counter);
	    			    		restart_counter_op(RESET); //reset_counter = 0
	    		                data_op(READ); //read 4 windows to data[]
	    		                data_op(RESET); //erase data
	    		             //   proceed_batch(); //compile full 64 s sample from flash
	    		                proceed_features(); //made features calculations
	    		               	lorasend(); //send to gw
	    		           	} else {
	    		           		get_window();  //collect data acquired by ULP
	    		           		data_op(SAVE);  // save to flash
	    		           		restart_counter_op(SAVE);  //inc restart counter
	    		           		ESP_LOGI(TAG, "Save restart counter:%d",restart_counter);

	    		           	}

	    	}

        switch (op_mode){
        	case RAW:
        		ESP_LOGI(TAG, "Entereing RAW mode loop");
        	    xTaskCreate(&udp_sender, "udp_sender", 4096, NULL, 5, &xTaskToNotify);
        		ulp_isr_sem = xSemaphoreCreateBinary();
        		assert(ulp_isr_sem);
        		esp_err_t err = rtc_isr_register(&ulp_isr, (void*) ulp_isr_sem, RTC_CNTL_SAR_INT_ST_M);
        		ESP_ERROR_CHECK(err);
        		REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
        		ESP_LOGI(TAG, "Starting ULP");
        		start_ulp_program();
        		while(1){
        			ESP_LOGI(TAG, "Waiting for ULP interrupt");
        			result = xSemaphoreTake(ulp_isr_sem, portMAX_DELAY);
        		    if (result == pdTRUE) {
        				    	        printf("ULP ISR triggered\n");
        				    	        restart_counter_op(READ);
        				    	        ESP_LOGI(TAG,"Restart counter :%d", restart_counter);
        				    	        get_window();  //collect data acquired by ULP
        				    	        if (xTaskToNotify!=NULL) {
        				    	        	xTaskNotify(xTaskToNotify,txpos,eSetValueWithOverwrite);
        				    	        }
        				    	        restart_counter_op(SAVE);  //inc restart counter
        				    	        start_ulp_program();
        				    	    } else {
        				    	        printf("ULP ISR timeout\n");
        				    	        esp_restart();
        				    	    };
        		     };
         		break;
        	case LoRa:
        		ESP_LOGI(TAG, "Going deep sleep");
        		fflush(stdout);
        		start_ulp_program();
        	    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
        	    /* Outdated API from old sdk ???
        	    rtc_gpio_isolate(GPIO_NUM_4);
        	    rtc_gpio_isolate(GPIO_NUM_15);
        	    rtc_gpio_isolate(GPIO_NUM_25);*/
        	    esp_deep_sleep_start();
        		break;
        }
}

void proceed_features(){
	features_init(batch);
	for(int i=0;i<FEATURES_NUM;i++) {
		features[i] = features_calc[i]();
		printf("Feature #%d = %f\n",i,features[i]);
	}
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
	            .format_if_mount_failed = true
	            //.allocation_unit_size = CONFIG_WL_SECTOR_SIZE  Outdated API????
	    };
	esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
	if (err != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to mount FATFS");
	        return err;
	   }

	switch(cmd){
	case SAVE:
		f = fopen("/spiflash/data.bin", "ab");
	    if (f == NULL) {
		  ESP_LOGE(TAG, "Failed to open file for writing");
		  return ESP_FAIL;}
	    ESP_LOGI(TAG,"Writing %d bytes to file",sizeof(window));
	    fwrite(window,sizeof(window),1,f);
	    fclose(f);
	    ESP_LOGI(TAG, "File written");
	    break;
	case READ:
		f = fopen("/spiflash/data.bin", "rb");
		if (f == NULL) {
		  ESP_LOGE(TAG, "Failed to open file for writing");
		return ESP_FAIL;}
		ESP_LOGI(TAG,"Reading %d bytes from file",sizeof(batch));
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
	static const char *TAG = "lora_send";
	lora_init();
	lora_set_frequency(433e6);
   	lora_enable_crc();
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


}

static void start_ulp_program()
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
}


static void ulp_isr(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xSemaphoreGiveFromISR(ulp_isr_sem,&xHigherPriorityTaskWoken);

}


void get_window(){
	        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, (esp_adc_cal_characteristics_t *)&adc_chars);
			for (int i=0;i<NO_OF_SAMPLES;i++){
			window[i][0] = esp_adc_cal_raw_to_voltage((&ulp_channel_0_measurements)[i] & UINT16_MAX, &adc_chars)-1600;
			window[i][1] = esp_adc_cal_raw_to_voltage((&ulp_channel_1_measurements)[i] & UINT16_MAX, &adc_chars)-1600;
			window[i][2] = esp_adc_cal_raw_to_voltage((&ulp_channel_2_measurements)[i] & UINT16_MAX, &adc_chars)-1600;
			//printf("%d;%d;%d\n",window[i][0],window[i][1],window[i][2]);
		}
}


esp_err_t event_handler(void *ctx, system_event_t *event)
{    static const char *TAG = "WiFi event handler";

	 switch(event->event_id) {
	    case SYSTEM_EVENT_STA_START:
	        esp_wifi_connect();
	        ESP_LOGI(TAG,"WiFi connected");
	        break;
	    case SYSTEM_EVENT_STA_GOT_IP:
	        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
	        ESP_LOGI(TAG,"WiFi got IP");
	        sprintf(my_ip,IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
	        break;
	    case SYSTEM_EVENT_STA_DISCONNECTED:
	        /* This is a workaround as ESP32 WiFi libs don't currently
	           auto-reassociate. */
	        esp_wifi_connect();
	        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
	        break;
	    default:
	        break;
	    }
	    return ESP_OK;
}

static void initialise_wifi(void)
{   static const char *TAG = "init_wifi";
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

mode_t check_mode(){
	return RAW;
}

static void udp_sender(void *pvParameters){

	 static const char *TAG = "sender_thread";

	 int socket_fd;

	 unsigned long st;

	 uint32_t ulNotifiedValue;


	 struct sockaddr_in sa,ra;

	 int sent_data;

	 ESP_LOGI(TAG, "Waiting for AP connection...");
     initialise_wifi();
	 xEventGroupWaitBits(wifi_event_group, IPV4_GOTIP_BIT, false, true, portMAX_DELAY);
	 ESP_LOGI(TAG, "Connected to AP");

	 /* Creates an UDP socket (SOCK_DGRAM) with Internet Protocol Family (PF_INET).
	  * Protocol family and Address family related. For example PF_INET Protocol Family and AF_INET family are coupled.
	 */
	 socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

	 if ( socket_fd < 0 )
	 {

	     ESP_LOGI(TAG,"socket call failed");
	     return;

	 }

	 memset(&sa, 0, sizeof(struct sockaddr_in));

	 sa.sin_family = AF_INET;
	 sa.sin_addr.s_addr = inet_addr(my_ip);
	 sa.sin_port = htons(SENDER_PORT_NUM);


	 /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
	 * machines IP address (Its defined by SENDER_IP_ADDR).
	 * Once bind is successful for UDP sockets application can operate
	 * on the socket descriptor for sending or receiving data.
	 */
	 if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
	 {
	   printf("Bind to Port Number %d ,IP address %s failed\n",SENDER_PORT_NUM,my_ip /*SENDER_IP_ADDR*/);
	   close(socket_fd);
	   return;
	 }
	 printf("Bind to Port Number %d ,IP address %s SUCCESS!!!\n",SENDER_PORT_NUM,my_ip);



	 memset(&ra, 0, sizeof(struct sockaddr_in));
	 ra.sin_family = AF_INET;
	 ra.sin_addr.s_addr = inet_addr(RECEIVER_IP_ADDR);
	 ra.sin_port = htons(RECEIVER_PORT_NUM);

     while(1){
     printf("Start udp_sender loop\n");
     st = xTaskGetTickCount();
     xTaskNotifyWait(pdFALSE,ULONG_MAX,&ulNotifiedValue,portMAX_DELAY);
     printf("Got notification from main loop\n");

	 for (int i=0;i<NO_OF_SAMPLES;i++) {
	     sent_data = sendto(socket_fd, window[i],sizeof(uint32_t)*3,0,(struct sockaddr*)&ra,sizeof(ra));
	     vTaskDelay(20 / portTICK_PERIOD_MS);
	     //printf("sending sample #:%d\n",i);
		// sent_data = sendto(socket_fd, TAG,strlen(TAG),0,(struct sockaddr*)&ra,sizeof(ra));
	     if(sent_data < 0)
	     {
	        printf("send failed\n");

	     }

	  }
	 printf("UDP send proceed time:%lu\n",(xTaskGetTickCount()-st));
     }
 }


