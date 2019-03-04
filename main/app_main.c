// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../components/http_server/my_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "camera.h"
#include "bitmap.h"
#include "led.h"
#include "qr_recoginize.h"
static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx);
static void handle_jpg(http_context_t http_ctx, void* ctx);
static void handle_jpg_stream(http_context_t http_ctx, void* ctx);
static esp_err_t event_handler(void *ctx, system_event_t *event);
static void initialise_wifi(void);

static const char* TAG = "camera_demo";

static const char* STREAM_CONTENT_TYPE =
        "multipart/x-mixed-replace; boundary=123456789000000000000987654321";

static const char* STREAM_BOUNDARY = "--123456789000000000000987654321";

EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;
static camera_pixelformat_t s_pixel_format;

#define CAMERA_PIXEL_FORMAT CAMERA_PF_JPEG
#define CAMERA_FRAME_SIZE CAMERA_FS_SVGA


void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CONFIG_D0,
        .pin_d1 = CONFIG_D1,
        .pin_d2 = CONFIG_D2,
        .pin_d3 = CONFIG_D3,
        .pin_d4 = CONFIG_D4,
        .pin_d5 = CONFIG_D5,
        .pin_d6 = CONFIG_D6,
        .pin_d7 = CONFIG_D7,
        .pin_xclk = CONFIG_XCLK,
        .pin_pclk = CONFIG_PCLK,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,
        .pin_reset = CONFIG_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
    };

    camera_model_t camera_model;
    err = camera_probe(&camera_config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }

    if (camera_model == CAMERA_OV7725) {
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        ESP_LOGI(TAG, "Detected OV7725 camera, using %s bitmap format",
                CAMERA_PIXEL_FORMAT == CAMERA_PF_GRAYSCALE ?
                        "grayscale" : "RGB565");
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        if (s_pixel_format == CAMERA_PF_JPEG)
        camera_config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }

    camera_config.pixel_format = s_pixel_format;
    err = camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
//    databuf = (char *) malloc(BUF_SIZE);
    initialise_wifi();

    http_server_t server;
    http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();
    ESP_ERROR_CHECK( http_server_start(&http_options, &server) );

    if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
        ESP_ERROR_CHECK( http_register_handler(server, "/pgm", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_grayscale_pgm, &camera_config) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/pgm for a single image/x-portable-graymap image", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_RGB565) {
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp_stream, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp_stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_JPEG) {
        ESP_ERROR_CHECK( http_register_handler(server, "/jpg", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/jpg_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg_stream, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/jpg_stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
    }
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");

}

static esp_err_t write_frame(http_context_t http_ctx)
{
    http_buffer_t fb_data = {
            .data = camera_get_fb(),
            .size = camera_get_data_size(),
            .data_is_persistent = true
    };
    return http_response_write(http_ctx, &fb_data);
}

static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx)
{
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        return;
    }
    char* pgm_header_str;
    asprintf(&pgm_header_str, "P5 %d %d %d\n",
            camera_get_fb_width(), camera_get_fb_height(), 255);
    if (pgm_header_str == NULL) {
        return;
    }

    size_t response_size = strlen(pgm_header_str) + camera_get_data_size();
    http_response_begin(http_ctx, 200, "image/x-portable-graymap", response_size);
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.pgm");
    http_buffer_t pgm_header = { .data = pgm_header_str };
    http_response_write(http_ctx, &pgm_header);
    free(pgm_header_str);

    write_frame(http_ctx);
    http_response_end(http_ctx);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
#if CONFIG_QR_RECOGNIZE
    camera_config_t *camera_config = ctx;
    xTaskCreate(qr_recoginze, "qr_recoginze", 111500, camera_config, 5, NULL);
#endif
}

static void handle_rgb_bmp(http_context_t http_ctx, void* ctx)
{
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        return;
    }

    bitmap_header_t* header = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
    if (header == NULL) {
        return;
    }

    http_response_begin(http_ctx, 200, "image/bmp", sizeof(*header) + camera_get_data_size());
    http_buffer_t bmp_header = {
            .data = header,
            .size = sizeof(*header)
    };
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.bmp");
    http_response_write(http_ctx, &bmp_header);
    free(header);

    write_frame(http_ctx);
    http_response_end(http_ctx);
}

static void handle_jpg(http_context_t http_ctx, void* ctx)
{
	if(get_light_state())
		led_open();
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        return;
    }

    http_response_begin(http_ctx, 200, "image/jpeg", camera_get_data_size());
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.jpg");
    write_frame(http_ctx);
    http_response_end(http_ctx);
    led_close();
}


static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx)
{
    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);
    bitmap_header_t* header = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
    if (header == NULL) {
        return;
    }
    http_buffer_t bmp_header = {
            .data = header,
            .size = sizeof(*header)
    };


    while (true) {
        esp_err_t err = camera_run();
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
            return;
        }

        err = http_response_begin_multipart(http_ctx, "image/bitmap",
                camera_get_data_size() + sizeof(*header));
        if (err != ESP_OK) {
            break;
        }
        err = http_response_write(http_ctx, &bmp_header);
        if (err != ESP_OK) {
            break;
        }
        err = write_frame(http_ctx);
        if (err != ESP_OK) {
            break;
        }
        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);
        if (err != ESP_OK) {
            break;
        }
    }

    free(header);
    http_response_end(http_ctx);
}

static void handle_jpg_stream(http_context_t http_ctx, void* ctx)
{
    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);
    if(get_light_state())
    		led_open();
    while (true) {
        esp_err_t err = camera_run();
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
            return;
        }
        err = http_response_begin_multipart(http_ctx, "image/jpg",
                camera_get_data_size());
        if (err != ESP_OK) {
            break;
        }
        err = write_frame(http_ctx);
        if (err != ESP_OK) {
            break;
        }
        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);
        if (err != ESP_OK) {
            break;
        }
    }
    http_response_end(http_ctx);
    led_close();
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

