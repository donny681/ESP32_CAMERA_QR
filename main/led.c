/*
 * led.c
 *
 *  Created on: 2017年12月11日
 *      Author: ai-thinker
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "led.h"
static bool light_state = true;
//void led_init() {
//	gpio_pad_select_gpio(LED_GPIO);
//	/* Set the GPIO as a push/pull output */
//	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
//}
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LED_GPIO 4
#define LEDC_TEST_DUTY 			300
	ledc_channel_config_t ledc_channel = { .channel = LEDC_CHANNEL_1, .duty = 0,
				.gpio_num = LED_GPIO, .speed_mode = LEDC_LS_MODE, .timer_sel =
						LEDC_LS_TIMER };
void led_init() {
	int ch;

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = { .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
			.freq_hz = 5000,                      // frequency of PWM signal
			.speed_mode = LEDC_LS_MODE,           // timer mode
			.timer_num = LEDC_LS_TIMER            // timer index
			};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	/*
	 * Prepare individual configuration
	 * for each channel of LED Controller
	 * by selecting:
	 * - controller's channel number
	 * - output duty cycle, set initially to 0
	 * - GPIO number where LED is connected to
	 * - speed mode, either high or low
	 * - timer servicing selected channel
	 *   Note: if different channels use one timer,
	 *         then frequency and bit_num of these channels
	 *         will be the same
	 */

	ledc_channel_config(&ledc_channel);
	ledc_fade_func_install(0);


}

void led_open() {
	ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel,
				LEDC_TEST_DUTY);
		ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
}

void led_close() {
	ledc_stop(ledc_channel.speed_mode, ledc_channel.channel, 0);
}

void open_light(void) {
	light_state = true;
}

void close_light(void) {
	light_state = false;
}

bool get_light_state(void) {
	return light_state;
}
