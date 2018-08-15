// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
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

#pragma once

#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAMERA_PF_RGB565 = 0,       //!< RGB, 2 bytes per pixel (not implemented)
    CAMERA_PF_YUV422 = 1,       //!< YUYV, 2 bytes per pixel (not implemented)
    CAMERA_PF_GRAYSCALE = 2,    //!< 1 byte per pixel
    CAMERA_PF_JPEG = 3,         //!< JPEG compressed
} camera_pixelformat_t;

typedef enum {
    CAMERA_FS_QQVGA = 4,     //!< 160x120
    CAMERA_FS_QVGA = 8,      //!< 320x240
    CAMERA_FS_VGA = 10,      //!< 640x480
    CAMERA_FS_SVGA = 11,     //!< 800x600
	CAMERA_FS_SXGA=12,		//	1280* 1024
	CAMERA_FS_UXGA=13,		//1600*1200
} camera_framesize_t;

typedef enum {
    CAMERA_NONE = 0,
    CAMERA_UNKNOWN = 1,
    CAMERA_OV7725 = 7725,
    CAMERA_OV2640 = 2640,
} camera_model_t;

typedef struct {
    int pin_reset;          /*!< GPIO pin for camera reset line */
    int pin_xclk;           /*!< GPIO pin for camera XCLK line */
    int pin_sscb_sda;       /*!< GPIO pin for camera SDA line */
    int pin_sscb_scl;       /*!< GPIO pin for camera SCL line */
    int pin_d7;             /*!< GPIO pin for camera D7 line */
    int pin_d6;             /*!< GPIO pin for camera D6 line */
    int pin_d5;             /*!< GPIO pin for camera D5 line */
    int pin_d4;             /*!< GPIO pin for camera D4 line */
    int pin_d3;             /*!< GPIO pin for camera D3 line */
    int pin_d2;             /*!< GPIO pin for camera D2 line */
    int pin_d1;             /*!< GPIO pin for camera D1 line */
    int pin_d0;             /*!< GPIO pin for camera D0 line */
    int pin_vsync;          /*!< GPIO pin for camera VSYNC line */
    int pin_href;           /*!< GPIO pin for camera HREF line */
    int pin_pclk;           /*!< GPIO pin for camera PCLK line */

    int xclk_freq_hz;       /*!< Frequency of XCLK signal, in Hz */

    ledc_timer_t ledc_timer;        /*!< LEDC timer to be used for generating XCLK  */
    ledc_channel_t ledc_channel;    /*!< LEDC channel to be used for generating XCLK  */

    camera_pixelformat_t pixel_format;
    camera_framesize_t frame_size;

    int jpeg_quality;
} camera_config_t;

#define ESP_ERR_CAMERA_BASE 0x20000
#define ESP_ERR_CAMERA_NOT_DETECTED             (ESP_ERR_CAMERA_BASE + 1)
#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ESP_ERR_CAMERA_BASE + 2)
#define ESP_ERR_CAMERA_NOT_SUPPORTED            (ESP_ERR_CAMERA_BASE + 3)

/**
 * @brief Probe the camera
 * This function enables LEDC peripheral to generate XCLK signal,
 * detects the camera I2C address and detects camera model.
 *
 * @param config camera configuration parameters
 * @param[out] out_camera_model output, detected camera model
 * @return ESP_OK if camera was detected
 */
esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model);

/**
 * @brief Initialize the camera driver
 *
 * @note call camera_probe before calling this function
 *
 * This function configures camera over I2C interface,
 * allocates framebuffer and DMA buffers,
 * initializes parallel I2S input, and sets up DMA descriptors.
 *
 * Currently this function can only be called once and there is
 * no way to de-initialize this module.
 *
 * @param config  Camera configuration parameters
 * @return ESP_OK on success
 */
esp_err_t camera_init(const camera_config_t* config);

/**
 * Deinitialize the camera driver
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the driver hasn't been initialized yet
 */
esp_err_t camera_deinit();

/**
 * @brief Obtain the pointer to framebuffer allocated by camera_init function.
 *
 * @return pointer to framebuffer
 */
uint8_t* camera_get_fb();

/**
 * @brief Return the size of valid data in the framebuffer
 *
 * For grayscale mode, this function returns width * height of the framebuffer.
 * For JPEG mode, this function returns the actual size of encoded JPEG image.
 * @return size of valid data in framebuffer, in bytes
 */
size_t camera_get_data_size();

/**
 * @brief Get the width of framebuffer, in pixels.
 * @return width of framebuffer, in pixels
 */
int camera_get_fb_width();

/**
 * @brief Get the height of framebuffer, in pixels.
 * @return height of framebuffer, in pixels
 */
int camera_get_fb_height();

/**
 * @brief Acquire one frame and store it into framebuffer
 *
 * This function waits for the next VSYNC, starts DMA to get data from camera,
 * and blocks until all lines of the image are stored into the framebuffer.
 * Once all lines are stored, the function returns.
 *
 * @return ESP_OK on success
 */
esp_err_t camera_run();

/**
 * @brief Print contents of framebuffer on terminal
 *
 */
void camera_print_fb();


#ifdef __cplusplus
}
#endif
