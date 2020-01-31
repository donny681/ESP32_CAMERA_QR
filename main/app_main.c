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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "bitmap.h"
#include "led.h"
#include "qr_recoginize.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>

static const char* TAG = "camera_qr_demo";

static httpd_handle_t start_webserver(void);
static void connect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data);
static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data);

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_BMP_PART = "Content-Type: image/bitmap\r\nContent-Length: %u\r\n\r\n";
static const char* _STREAM_JPG_PART = "Content-Type: image/jpg\r\nContent-Length: %u\r\n\r\n";

void app_main()
{
    static httpd_handle_t server = NULL;

    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    static camera_config_t camera_config = {
        .pin_pwdn  = -1,                        // power down is not used
        .pin_reset = CONFIG_RESET,              // software reset will be performed
        .pin_xclk = CONFIG_XCLK,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,

        .pin_d7 = CONFIG_D7,
        .pin_d6 = CONFIG_D6,
        .pin_d5 = CONFIG_D5,
        .pin_d4 = CONFIG_D4,
        .pin_d3 = CONFIG_D3,
        .pin_d2 = CONFIG_D2,
        .pin_d1 = CONFIG_D1,
        .pin_d0 = CONFIG_D0,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_pclk = CONFIG_PCLK,

        //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE, /*PIXFORMAT_RGB888,*/ 
        .frame_size = FRAMESIZE_QQVGA,      //QQVGA-QXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = 12, //0-63 lower number means higher quality
        .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
    };

    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();

    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");
}

#define BUFFER_LEN 512

/* Convert the pgm gray in a rgb bitmap */
static esp_err_t write_gray_frame(httpd_req_t *req, camera_fb_t * fb)
{
char* buf;
int x = 0;
int size;
esp_err_t err = ESP_OK;
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    /* To save RAM send the converted image in chunks of 512 bytes. */
    buf = malloc( BUFFER_LEN * 3 );
    
    if (!buf ) {
        ESP_LOGE(TAG, "Dinamic memory failed");
        return ESP_FAIL;    
    }

    while ( (x<fb->len) && (err == ESP_OK) ) {
        size = (fb->len >= BUFFER_LEN) ? BUFFER_LEN : fb->len;       

        /* To convert, match the RGB bytes to the value of the PGM byte. */
        for (int i=0; i<size; i++) {
            buf[i * 3 ] = fb->buf[i + x];
            buf[(i * 3) + 1 ] = fb->buf[i + x];
            buf[(i * 3) + 2 ] = fb->buf[i + x];        
        }

        err = httpd_resp_send_chunk(req, buf, size * 3);
        x += size;
    }

    free( buf );

    return err;
}

/* HTTP pgm handler to take one picture and file download */
static esp_err_t handle_grayscale_pgm(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;
    
    char pgm_header_str[64];
    
    // acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    err = httpd_resp_set_type(req, "image/x-portable-graymap");

    if (err == ESP_OK){
        err = httpd_resp_set_hdr(req, "Content-disposition", "inline; filename=capture.pgm");
    }

    if (err == ESP_OK){
        size_t hlen = snprintf((char *)pgm_header_str, 64, "P5 %d %d %d\n", fb->width, fb->height, 255);

        err = httpd_resp_send_chunk(req, (const char *)pgm_header_str, hlen);
    }

    if (err == ESP_OK){
        err = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }

    /* buf_len as 0 to mark that all chunks have been sent.  */
    if (err == ESP_OK){
        err = httpd_resp_send_chunk(req, 0, 0);
    }
    
#if CONFIG_QR_RECOGNIZE
    xTaskCreate(qr_recoginze, "qr_recoginze", 111500, fb, 5, NULL);
#else
    esp_camera_fb_return(fb);
#endif
    
    return err;
}

/* HTTP bmp handler to take one picture*/
static esp_err_t handle_rgb_bmp(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;

    // acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    sensor_t * sensor = esp_camera_sensor_get();
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    bitmap_header_t* header = bmp_create_header(fb->width, fb->height);
    if (header == NULL) {
        return ESP_FAIL;
    }

    err = httpd_resp_set_type(req, "image/bmp");
    
    if (err == ESP_OK){
        err = httpd_resp_set_hdr(req, "Content-disposition", "inline; filename=capture.bmp");
    }

    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, (const char*)header, sizeof(*header));
    }

    free(header);

    if (err == ESP_OK) {
        /* convert an image with a gray format of 8 bits to a 24 bit bmp. */        
        if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
            err = write_gray_frame(req, fb);
        }else{
            err = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        }
    }

    /* buf_len as 0 to mark that all chunks have been sent. */
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, 0, 0);
    }

#if CONFIG_QR_RECOGNIZE
    xTaskCreate(qr_recoginze, "qr_recoginze", 111500, fb, 5, NULL);
#else
    esp_camera_fb_return(fb);
#endif

    return err;
}

/* HTTP jpg handler to take one picture */
static esp_err_t handle_jpg(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;
    
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    err = httpd_resp_set_type(req, "image/jpeg");

    if (err == ESP_OK) {
        err = httpd_resp_set_hdr(req, "Content-disposition", "inline; filename=capture.jpg");
    }

    if (err == ESP_OK) {
       err = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);

    return err;
}

/* HTTP bmp stream handler */
static esp_err_t handle_rgb_bmp_stream(httpd_req_t *req)
{
    char * part_buf[64];
    esp_err_t err = ESP_OK;

    camera_fb_t * fb = esp_camera_fb_get();
    sensor_t * sensor = esp_camera_sensor_get();

    bitmap_header_t* header = bmp_create_header(fb->width, fb->height);
    if (header == NULL) {
        return ESP_FAIL;
    }

    err = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    esp_camera_fb_return(fb);

    while (err == ESP_OK) {
        fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            err = ESP_FAIL;            
        }

        if (err == ESP_OK) {
            int len = fb->len;

            if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
                len *= 3;
            } 
            
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_BMP_PART, len + sizeof(*header));

            err = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }

        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, (const char*)header, sizeof(*header));
        }

        if (err == ESP_OK) {
            /* convert an image with a gray format of 8 bits to a 24 bit bmp. */            
            if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
                err = write_gray_frame(req, fb);
            }else{
                err = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
            }
        }
        
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        esp_camera_fb_return(fb);
    }

    free(header);
 
    return err;
}

/* HTTP jpg stream handler */
static esp_err_t handle_jpg_stream(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;
    char * part_buf[64];

    err = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while (err == ESP_OK) {
        //acquire a frame
        camera_fb_t * fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            err = ESP_FAIL;
        }

        if (err == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_JPG_PART, fb->len);

            err = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }

        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        }
        
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        esp_camera_fb_return(fb);
    }
    
    return err;
}

static const httpd_uri_t bmp = {
    .uri       = "/bmp",
    .method    = HTTP_GET,
    .handler   = handle_rgb_bmp,
};

static const httpd_uri_t bmp_stream = {
    .uri       = "/bmp_stream",
    .method    = HTTP_GET,
    .handler   = handle_rgb_bmp_stream,
};

static const httpd_uri_t pgm = {
    .uri       = "/pgm",
    .method    = HTTP_GET,
    .handler   = handle_grayscale_pgm,
};

static const httpd_uri_t jpg = {
    .uri       = "/jpg",
    .method    = HTTP_GET,
    .handler   = handle_jpg,
};

static const httpd_uri_t jpg_stream = {
    .uri       = "/jpg_stream",
    .method    = HTTP_GET,
    .handler   = handle_jpg_stream,
};

static ip4_addr_t get_ip_addr(void)
{
    tcpip_adapter_ip_info_t ip_info; 
   	
    // IP address.
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);

    return ip_info.ip;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");

        sensor_t * sensor = esp_camera_sensor_get();

        ip4_addr_t s_ip_addr = get_ip_addr();
       
        if (sensor->pixformat == PIXFORMAT_GRAYSCALE) {
            httpd_register_uri_handler(server, &bmp);
            httpd_register_uri_handler(server, &bmp_stream);
            httpd_register_uri_handler(server, &pgm);

            ESP_LOGI(TAG, "Open http://" IPSTR "/bmp for a single image/bmp gray image", IP2STR(&s_ip_addr));
            ESP_LOGI(TAG, "Open http://" IPSTR "/bmp_stream for multipart/x-mixed-replace stream of gray bitmaps", IP2STR(&s_ip_addr));
            ESP_LOGI(TAG, "Open http://" IPSTR "/pgm for a single image/x-portable-graymap image", IP2STR(&s_ip_addr));
        } else if (sensor->pixformat == PIXFORMAT_RGB888) {
            httpd_register_uri_handler(server, &bmp);
            httpd_register_uri_handler(server, &bmp_stream);
            
            ESP_LOGI(TAG, "Open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
            ESP_LOGI(TAG, "Open http://" IPSTR "/bmp_stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
        } else if (sensor->pixformat == PIXFORMAT_JPEG) {
            httpd_register_uri_handler(server, &jpg);  
            httpd_register_uri_handler(server, &jpg_stream);          
     
            ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
            ESP_LOGI(TAG, "Open http://" IPSTR "/jpg_stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
        }

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        httpd_stop(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, 
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;

    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

