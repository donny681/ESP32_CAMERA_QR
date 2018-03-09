/*
 * mygpio.c
 *
 *  Created on: 2018年3月9日
 *      Author: ai-thinker
 */
#include "factorytest.h"
#include "esp_log.h"
#include "camera.h"
#include "led.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
static const char* TAG = "camera_demo";
static int counter = 0;

//void failed_led
int test_mode = 0; //0是测试模式，1是正常模式
void jpeg_test(uint8_t *data, size_t size) {
	nvs_handle my_handle;
	int i = 0, j = 0;
	for (i = 0; i < size; i++) {
		if ((data[i] == 0xff) && (data[i + 1] == 0xd8)) {
			j++;
			break;
		}
	}
	for (; i < size; i++) {
		if ((data[i] == 0xff) && (data[i+1] == 0xd9)) {
			j++;
			break;
		}
	}
	if (j >= 2) {

		ESP_LOGE(TAG, "test successfully");
		while (1) {
			i++;
			vTaskDelay(500 / portTICK_PERIOD_MS);
			if (i % 2 == 0) {
//				printf("Opening light. \r\n");
				led_open();
			} else {
//				printf("Close light\r\n");
				led_close();
			}
		}
	} else {
		printf("Opening Non-Volatile Storage (NVS) handle... \r\n");
		esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Error (%d) opening NVS handle!\n", err);
		} else {
			int32_t counter = 0; // value will default to 0, if not set yet in NVS
			err = nvs_get_i32(my_handle, "restart_counter", &counter);
			ESP_LOGE(TAG, "Camera test=%d", counter);
			if (counter <= 2) {
				counter++;
				err = nvs_set_i32(my_handle, "restart_counter", counter);
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				err = nvs_commit(my_handle);
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				nvs_close(my_handle);
				vTaskDelay(150 / portTICK_PERIOD_MS);
				esp_restart();
			} else {
				counter = 0;
				err = nvs_set_i32(my_handle, "restart_counter", counter);
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				err = nvs_commit(my_handle);
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				nvs_close(my_handle);
				led_open();

			}
		}
	}

}

void gpio_init() {
//	gpio_pad_select_gpio(TEST_GPIO);
//	gpio_set_direction(TEST_GPIO, GPIO_MODE_INPUT);
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = 1 << TEST_GPIO;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 1;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
}

