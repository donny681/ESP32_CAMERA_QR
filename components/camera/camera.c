/*
 * Portions of this file come from OpenMV project (see sensor_* functions in the end of file)
 * Here is the copyright for these parts:
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 *
 * Rest of the functions are licensed under Apache license as found below:
 */

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "sensor.h"
#include "sccb.h"
#include "wiring.h"
#include "camera.h"
#include "camera_common.h"
#include "xclk.h"
#include "twi.h"
#if CONFIG_OV2640_SUPPORT
#include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif

#define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

static const char* TAG = "camera";

camera_state_t* s_state = NULL;

const int resolution[][2] = { { 40, 30 }, /* 40x30 */
{ 64, 32 }, /* 64x32 */
{ 64, 64 }, /* 64x64 */
{ 88, 72 }, /* QQCIF */
{ 160, 120 }, /* QQVGA */
{ 128, 160 }, /* QQVGA2*/
{ 176, 144 }, /* QCIF  */
{ 240, 160 }, /* HQVGA */
{ 320, 240 }, /* QVGA  */
{ 352, 288 }, /* CIF   */
{ 640, 480 }, /* VGA   */
{ 800, 600 }, /* SVGA  */
{ 1280, 1024 }, /* SXGA  */
{ 1600, 1200 }, /* UXGA  */
};

static void i2s_init();
static void i2s_run();
static void IRAM_ATTR gpio_isr(void* arg);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void dma_desc_deinit();
static void dma_filter_task(void *pvParameters);
static void dma_filter_grayscale(const dma_elem_t* src, lldesc_t* dma_desc,
		uint8_t* dst);
static void dma_filter_grayscale_highspeed(const dma_elem_t* src,
		lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc,
		uint8_t* dst);
static void dma_filter_rgb565(const dma_elem_t* src, lldesc_t* dma_desc,
		uint8_t* dst);
static void i2s_stop();

static bool is_hs_mode() {
	return s_state->config.xclk_freq_hz > 10000000;
}

static size_t i2s_bytes_per_sample(i2s_sampling_mode_t mode) {
	switch (mode) {
	case SM_0A00_0B00:
		return 4;
	case SM_0A0B_0B0C:
		return 4;
	case SM_0A0B_0C0D:
		return 2;
	default:
		assert(0 && "invalid sampling mode");
		return 0;
	}
}

esp_err_t camera_probe(const camera_config_t* config,
		camera_model_t* out_camera_model) {
	if (s_state != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
	if (!s_state) {
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGD(TAG, "Enabling XCLK output");
	camera_enable_out_clock(config);

	ESP_LOGD(TAG, "Initializing SSCB");
	SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);

	ESP_LOGD(TAG, "Resetting camera");
	gpio_config_t conf = { 0 };
	conf.pin_bit_mask = 1LL << config->pin_reset;
	conf.mode = GPIO_MODE_OUTPUT;
	gpio_config(&conf);

	gpio_set_level(config->pin_reset, 1);
	delay(3000);

	gpio_set_level(config->pin_reset, 0);
	delay(1000);

#if CONFIG_OV2640_SUPPORT
	uint8_t buf[] = {0xff, 0x01};
	twi_writeTo(0x30, buf, 2, true);
#endif

	ESP_LOGD(TAG, "Searching for camera address");
	/* Probe the sensor */
	delay(10);
	uint8_t slv_addr = SCCB_Probe();
	if (slv_addr == 0) {
		*out_camera_model = CAMERA_NONE;
		return ESP_ERR_CAMERA_NOT_DETECTED;
	}
	s_state->sensor.slv_addr = slv_addr;
	ESP_LOGD(TAG, "Detected camera at address=0x%02x", slv_addr);
	sensor_id_t* id = &s_state->sensor.id;
	id->PID = SCCB_Read(slv_addr, REG_PID);
	id->VER = SCCB_Read(slv_addr, REG_VER);
	id->MIDL = SCCB_Read(slv_addr, REG_MIDL);
	id->MIDH = SCCB_Read(slv_addr, REG_MIDH);
	delay(10);
	ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
			id->PID, id->VER, id->MIDH, id->MIDL);

	switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
	case OV2640_PID:
	*out_camera_model = CAMERA_OV2640;
	ov2640_init(&s_state->sensor);
	break;
#endif
#if CONFIG_OV7725_SUPPORT
	case OV7725_PID:
	*out_camera_model = CAMERA_OV7725;
	ov7725_init(&s_state->sensor);
	break;
#endif
	default:
		id->PID = 0;
		*out_camera_model = CAMERA_UNKNOWN;
		ESP_LOGD(TAG, "Detected camera not supported.")
		;
		return ESP_ERR_CAMERA_NOT_SUPPORTED;
	}

	ESP_LOGD(TAG, "Doing SW reset of sensor");
	s_state->sensor.reset(&s_state->sensor);

	return ESP_OK;
}

esp_err_t camera_init(const camera_config_t* config) {
	if (!s_state) {
		return ESP_ERR_INVALID_STATE;
	}
	if (s_state->sensor.id.PID == 0) {
		return ESP_ERR_CAMERA_NOT_SUPPORTED;
	}
	memcpy(&s_state->config, config, sizeof(*config));
	esp_err_t err = ESP_OK;
	framesize_t frame_size = (framesize_t) config->frame_size;
	pixformat_t pix_format = (pixformat_t) config->pixel_format;
	s_state->width = resolution[frame_size][0];
	s_state->height = resolution[frame_size][1];
	s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

	ESP_LOGD(TAG, "Setting frame size to %dx%d", s_state->width,
			s_state->height);
	if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
		ESP_LOGE(TAG, "Failed to set frame size");
		err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
		goto fail;
	}
	s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

#if ENABLE_TEST_PATTERN
	/* Test pattern may get handy
	 if you are unable to get the live image right.
	 Once test pattern is enable, sensor will output
	 vertical shaded bars instead of live image.
	 */
	s_state->sensor.set_colorbar(&s_state->sensor, 1);
	ESP_LOGD(TAG, "Test pattern enabled");
#endif

	if (pix_format == PIXFORMAT_GRAYSCALE) {
//		if (s_state->sensor.id.PID != OV7725_PID) {
//			ESP_LOGE(TAG, "Grayscale format is only supported for ov7225");
//			err = ESP_ERR_NOT_SUPPORTED;
//			goto fail;
//		}
		s_state->fb_size = s_state->width * s_state->height;
		if (is_hs_mode()) {
			s_state->sampling_mode = SM_0A0B_0B0C;
			s_state->dma_filter = &dma_filter_grayscale_highspeed;
		} else {
			s_state->sampling_mode = SM_0A0B_0C0D;
			s_state->dma_filter = &dma_filter_grayscale;
		}
		s_state->in_bytes_per_pixel = 2;       // camera sends YUYV
		s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
	} else if (pix_format == PIXFORMAT_RGB565) {
//		if (s_state->sensor.id.PID != OV7725_PID) {
//			ESP_LOGE(TAG, "RGB565 format is only supported for ov7225");
//			err = ESP_ERR_NOT_SUPPORTED;
//			goto fail;
//		}
		s_state->fb_size = s_state->width * s_state->height * 3;
		if (is_hs_mode()) {
			s_state->sampling_mode = SM_0A0B_0B0C;
		} else {
			s_state->sampling_mode = SM_0A00_0B00;
		}
		s_state->in_bytes_per_pixel = 2;       // camera sends RGB565 (2 bytes)
		s_state->fb_bytes_per_pixel = 3;       // frame buffer stores RGB888
		s_state->dma_filter = &dma_filter_rgb565;

	} else if (pix_format == PIXFORMAT_JPEG) {
		if (s_state->sensor.id.PID != OV2640_PID) {
			ESP_LOGE(TAG, "JPEG format is only supported for ov2640");
			err = ESP_ERR_NOT_SUPPORTED;
			goto fail;
		}
		int qp = config->jpeg_quality;
		int compression_ratio_bound;
		if (qp >= 30) {
			compression_ratio_bound = 5;
		} else if (qp >= 10) {
			compression_ratio_bound = 10;
		} else {
			compression_ratio_bound = 20;
		}
		(*s_state->sensor.set_quality)(&s_state->sensor, qp);
		size_t equiv_line_count = s_state->height / compression_ratio_bound;
		s_state->fb_size = s_state->width * equiv_line_count * 2 /* bpp */;
		s_state->dma_filter = &dma_filter_jpeg;
		if (is_hs_mode()) {
			s_state->sampling_mode = SM_0A0B_0B0C;
		} else {
			s_state->sampling_mode = SM_0A00_0B00;
		}
		s_state->in_bytes_per_pixel = 2;
		s_state->fb_bytes_per_pixel = 2;
	} else {
		ESP_LOGE(TAG, "Requested format is not supported");
		err = ESP_ERR_NOT_SUPPORTED;
		goto fail;
	}

	ESP_LOGD(TAG,
			"in_bpp: %d, fb_bpp: %d, fb_size: %d, mode: %d, width: %d height: %d",
			s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel,
			s_state->fb_size, s_state->sampling_mode, s_state->width,
			s_state->height);

	ESP_LOGD(TAG, "Allocating frame buffer (%d bytes)", s_state->fb_size);
	s_state->fb = (uint8_t*) calloc(s_state->fb_size, 1);
	if (s_state->fb == NULL) {
		ESP_LOGE(TAG, "Failed to allocate frame buffer");
		err = ESP_ERR_NO_MEM;
		goto fail;
	}

	ESP_LOGD(TAG, "Initializing I2S and DMA");
	i2s_init();
	err = dma_desc_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize I2S and DMA");
		goto fail;
	}

	s_state->data_ready = xQueueCreate(16, sizeof(size_t));
	s_state->frame_ready = xSemaphoreCreateBinary();
	if (s_state->data_ready == NULL || s_state->frame_ready == NULL) {
		ESP_LOGE(TAG, "Failed to create semaphores");
		err = ESP_ERR_NO_MEM;
		goto fail;
	}
	if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10,
			&s_state->dma_filter_task, 1)) {
		ESP_LOGE(TAG, "Failed to create DMA filter task");
		err = ESP_ERR_NO_MEM;
		goto fail;
	}

	ESP_LOGD(TAG, "Initializing GPIO interrupts");
	gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
	gpio_intr_enable(s_state->config.pin_vsync);
	err = gpio_isr_register(&gpio_isr, (void*) TAG,
	ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_IRAM,
			&s_state->vsync_intr_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "gpio_isr_register failed (%x)", err);
		goto fail;
	}

	// skip at least one frame after changing camera settings
	while (gpio_get_level(s_state->config.pin_vsync) == 0) {
		;
	}
	while (gpio_get_level(s_state->config.pin_vsync) != 0) {
		;
	}
	while (gpio_get_level(s_state->config.pin_vsync) == 0) {
		;
	}
	s_state->frame_count = 0;
	ESP_LOGD(TAG, "Init done");
	return ESP_OK;

	fail: camera_deinit();
	return err;
}

esp_err_t camera_deinit() {
	if (s_state == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	if (s_state->dma_filter_task) {
		vTaskDelete(s_state->dma_filter_task);
	}
	if (s_state->data_ready) {
		vQueueDelete(s_state->data_ready);
	}
	if (s_state->frame_ready) {
		vSemaphoreDelete(s_state->frame_ready);
	}
	if (s_state->vsync_intr_handle) {
		esp_intr_disable(s_state->vsync_intr_handle);
		esp_intr_free(s_state->vsync_intr_handle);
	}
	if (s_state->i2s_intr_handle) {
		esp_intr_disable(s_state->i2s_intr_handle);
		esp_intr_free(s_state->i2s_intr_handle);
	}
	dma_desc_deinit();
	free(s_state->fb);
	free(s_state);
	s_state = NULL;
	camera_disable_out_clock();
	periph_module_disable(PERIPH_I2S0_MODULE);
	return ESP_OK;
}

uint8_t* camera_get_fb() {
	if (s_state == NULL) {
		return NULL;
	}
	return s_state->fb;
}

int camera_get_fb_width() {
	if (s_state == NULL) {
		return 0;
	}
	return s_state->width;
}

int camera_get_fb_height() {
	if (s_state == NULL) {
		return 0;
	}
	return s_state->height;
}

size_t camera_get_data_size() {
	if (s_state == NULL) {
		return 0;
	}
	return s_state->data_size;
}

esp_err_t camera_run() {
	if (s_state == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	struct timeval tv_start;
	gettimeofday(&tv_start, NULL);
#ifndef _NDEBUG
	memset(s_state->fb, 0, s_state->fb_size);
#endif // _NDEBUG
	i2s_run();
	ESP_LOGD(TAG, "Waiting for frame");
	xSemaphoreTake(s_state->frame_ready, portMAX_DELAY);
	struct timeval tv_end;
	gettimeofday(&tv_end, NULL);
	int time_ms = (tv_end.tv_sec - tv_start.tv_sec) * 1000
			+ (tv_end.tv_usec - tv_start.tv_usec) / 1000;
	ESP_LOGI(TAG, "Frame %d done in %d ms", s_state->frame_count, time_ms);
	s_state->frame_count++;
	return ESP_OK;
}

static esp_err_t dma_desc_init() {
	assert(s_state->width % 4 == 0);
	size_t line_size = s_state->width * s_state->in_bytes_per_pixel
			* i2s_bytes_per_sample(s_state->sampling_mode);
	ESP_LOGD(TAG, "Line width (for DMA): %d bytes", line_size);
	size_t dma_per_line = 1;
	size_t buf_size = line_size;
	while (buf_size >= 4096) {
		buf_size /= 2;
		dma_per_line *= 2;
	}
	size_t dma_desc_count = dma_per_line * 4;
	s_state->dma_buf_width = line_size;
	s_state->dma_per_line = dma_per_line;
	s_state->dma_desc_count = dma_desc_count;
	ESP_LOGD(TAG, "DMA buffer size: %d, DMA buffers per line: %d", buf_size,
			dma_per_line);
	ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);

	s_state->dma_buf = (dma_elem_t**) malloc(
			sizeof(dma_elem_t*) * dma_desc_count);
	if (s_state->dma_buf == NULL) {
		return ESP_ERR_NO_MEM;
	}
	s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
	if (s_state->dma_desc == NULL) {
		return ESP_ERR_NO_MEM;
	}
	size_t dma_sample_count = 0;
	for (int i = 0; i < dma_desc_count; ++i) {
		ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
		dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
		if (buf == NULL) {
			return ESP_ERR_NO_MEM;
		}
		s_state->dma_buf[i] = buf;
		ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);

		lldesc_t* pd = &s_state->dma_desc[i];
		pd->length = buf_size;
		if (s_state->sampling_mode == SM_0A0B_0B0C
				&& (i + 1) % dma_per_line == 0) {
			pd->length -= 4;
		}
		dma_sample_count += pd->length / 4;
		pd->size = pd->length;
		pd->owner = 1;
		pd->sosf = 1;
		pd->buf = (uint8_t*) buf;
		pd->offset = 0;
		pd->empty = 0;
		pd->eof = 1;
		pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
	}
	s_state->dma_done = false;
	s_state->dma_sample_count = dma_sample_count;
	return ESP_OK;
}

static void dma_desc_deinit() {
	if (s_state->dma_buf) {
		for (int i = 0; i < s_state->dma_desc_count; ++i) {
			free(s_state->dma_buf[i]);
		}
	}
	free(s_state->dma_buf);
	free(s_state->dma_desc);
}

static inline void i2s_conf_reset() {
	const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M
			| I2S_AHBM_FIFO_RST_M;
	I2S0.lc_conf.val |= lc_conf_reset_flags;
	I2S0.lc_conf.val &= ~lc_conf_reset_flags;

	const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
			| I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
	I2S0.conf.val |= conf_reset_flags;
	I2S0.conf.val &= ~conf_reset_flags;
	while (I2S0.state.rx_fifo_reset_back) {
		;
	}
}

static void i2s_init() {
	camera_config_t* config = &s_state->config;

	// Configure input GPIOs
	gpio_num_t pins[] = { config->pin_d7, config->pin_d6, config->pin_d5,
			config->pin_d4, config->pin_d3, config->pin_d2, config->pin_d1,
			config->pin_d0, config->pin_vsync, config->pin_href,
			config->pin_pclk };
	gpio_config_t conf = { .mode = GPIO_MODE_INPUT, .pull_up_en =
			GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE };
	for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
		conf.pin_bit_mask = 1LL << pins[i];
		gpio_config(&conf);
	}

	// Route input GPIOs to I2S peripheral using GPIO matrix
	gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
	gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
	gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
	gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
	gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
	gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
	gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
	gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
	gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);
	gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
	gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false);
	gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

	// Enable and configure I2S peripheral
	periph_module_enable(PERIPH_I2S0_MODULE);
	// Toggle some reset bits in LC_CONF register
	// Toggle some reset bits in CONF register
	i2s_conf_reset();
	// Enable slave mode (sampling clock is external)
	I2S0.conf.rx_slave_mod = 1;
	// Enable parallel mode
	I2S0.conf2.lcd_en = 1;
	// Use HSYNC/VSYNC/HREF to control sampling
	I2S0.conf2.camera_en = 1;
	// Configure clock divider
	I2S0.clkm_conf.clkm_div_a = 1;
	I2S0.clkm_conf.clkm_div_b = 0;
	I2S0.clkm_conf.clkm_div_num = 2;
	// FIFO will sink data to DMA
	I2S0.fifo_conf.dscr_en = 1;
	// FIFO configuration
	I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
	I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
	I2S0.conf_chan.rx_chan_mod = 1;
	// Clear flags which are used in I2S serial mode
	I2S0.sample_rate_conf.rx_bits_mod = 0;
	I2S0.conf.rx_right_first = 0;
	I2S0.conf.rx_msb_right = 0;
	I2S0.conf.rx_msb_shift = 0;
	I2S0.conf.rx_mono = 0;
	I2S0.conf.rx_short_sync = 0;
	I2S0.timing.val = 0;

	// Allocate I2S interrupt, keep it disabled
	esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
	ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
			&i2s_isr, NULL, &s_state->i2s_intr_handle);
}

static void i2s_stop() {
	esp_intr_disable(s_state->i2s_intr_handle);
	esp_intr_disable(s_state->vsync_intr_handle);
	i2s_conf_reset();
	I2S0.conf.rx_start = 0;
	size_t val = SIZE_MAX;
	BaseType_t higher_priority_task_woken;
	xQueueSendFromISR(s_state->data_ready, &val, &higher_priority_task_woken);
}

static void i2s_run() {
#ifndef _NDEBUG
	for (int i = 0; i < s_state->dma_desc_count; ++i) {
		lldesc_t* d = &s_state->dma_desc[i];
		ESP_LOGV(TAG, "DMA desc %2d: %u %u %u %u %u %u %p %p", i, d->length,
				d->size, d->offset, d->eof, d->sosf, d->owner, d->buf,
				d->qe.stqe_next);
		memset(s_state->dma_buf[i], 0, d->length);
	}
#endif

	// wait for vsync
	ESP_LOGD(TAG, "Waiting for positive edge on VSYNC");
	while (gpio_get_level(s_state->config.pin_vsync) == 0) {
		;
	}
	while (gpio_get_level(s_state->config.pin_vsync) != 0) {
		;
	}
	ESP_LOGD(TAG, "Got VSYNC");

	s_state->dma_done = false;
	s_state->dma_desc_cur = 0;
	s_state->dma_received_count = 0;
	s_state->dma_filtered_count = 0;
	esp_intr_disable(s_state->i2s_intr_handle);
	i2s_conf_reset();

	I2S0.rx_eof_num = s_state->dma_sample_count;
	I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
	I2S0.in_link.start = 1;
	I2S0.int_clr.val = I2S0.int_raw.val;
	I2S0.int_ena.val = 0;
	I2S0.int_ena.in_done = 1;
	esp_intr_enable(s_state->i2s_intr_handle);
	if (s_state->config.pixel_format == CAMERA_PF_JPEG) {
		esp_intr_enable(s_state->vsync_intr_handle);
	}
	I2S0.conf.rx_start = 1;

}

static void IRAM_ATTR signal_dma_buf_received(bool* need_yield) {
	size_t dma_desc_filled = s_state->dma_desc_cur;
	s_state->dma_desc_cur = (dma_desc_filled + 1) % s_state->dma_desc_count;
	s_state->dma_received_count++;
	BaseType_t higher_priority_task_woken;
	BaseType_t ret = xQueueSendFromISR(s_state->data_ready, &dma_desc_filled,
			&higher_priority_task_woken);
	if (ret != pdTRUE) {
		ESP_EARLY_LOGW(TAG, "queue send failed (%d), dma_received_count=%d",
				ret, s_state->dma_received_count);
	}
	*need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}

static void IRAM_ATTR i2s_isr(void* arg) {
	I2S0.int_clr.val = I2S0.int_raw.val;
	bool need_yield;
	signal_dma_buf_received(&need_yield);
	ESP_EARLY_LOGV(TAG, "isr, cnt=%d", s_state->dma_received_count);
	if (s_state->dma_received_count
			== s_state->height * s_state->dma_per_line) {
		i2s_stop();
	}
	if (need_yield) {
		portYIELD_FROM_ISR();
	}
}

static void IRAM_ATTR gpio_isr(void* arg) {
	uint32_t isr = GPIO.status;
	if (isr == 0)
		ESP_EARLY_LOGE(TAG, "isr=%d", isr);
	GPIO.status1_w1tc.val = GPIO.status1.val;
	GPIO.status_w1tc = GPIO.status;
	bool need_yield = false;
	ESP_EARLY_LOGV(TAG, "gpio isr, cnt=%d", s_state->dma_received_count);
	if (gpio_get_level(s_state->config.pin_vsync) == 0
			&& s_state->dma_received_count > 0 && !s_state->dma_done) {
		signal_dma_buf_received(&need_yield);
		i2s_stop();
	}
	if (need_yield) {
		portYIELD_FROM_ISR();
	}
}

static size_t get_fb_pos() {
	return s_state->dma_filtered_count * s_state->width
			* s_state->fb_bytes_per_pixel / s_state->dma_per_line;
}

static void IRAM_ATTR dma_filter_task(void *pvParameters) {
	while (true) {
		size_t buf_idx;
		xQueueReceive(s_state->data_ready, &buf_idx, portMAX_DELAY);
		if (buf_idx == SIZE_MAX) {
			s_state->data_size = get_fb_pos();
			xSemaphoreGive(s_state->frame_ready);
			continue;
		}

		uint8_t* pfb = s_state->fb + get_fb_pos();
		const dma_elem_t* buf = s_state->dma_buf[buf_idx];
		lldesc_t* desc = &s_state->dma_desc[buf_idx];
		ESP_LOGV(TAG, "dma_flt: pos=%d ", get_fb_pos());
		(*s_state->dma_filter)(buf, desc, pfb);
		s_state->dma_filtered_count++;
		ESP_LOGV(TAG, "dma_flt: flt_count=%d ", s_state->dma_filtered_count);
	}
}

static void IRAM_ATTR dma_filter_grayscale(const dma_elem_t* src,
		lldesc_t* dma_desc, uint8_t* dst) {
	assert(s_state->sampling_mode == SM_0A0B_0C0D);
	size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
	for (size_t i = 0; i < end; ++i) {
		// manually unrolling 4 iterations of the loop here
		dst[0] = src[0].sample1;
		dst[1] = src[1].sample1;
		dst[2] = src[2].sample1;
		dst[3] = src[3].sample1;
		src += 4;
		dst += 4;
	}
}

static void IRAM_ATTR dma_filter_grayscale_highspeed(const dma_elem_t* src,
		lldesc_t* dma_desc, uint8_t* dst) {
	assert(s_state->sampling_mode == SM_0A0B_0B0C);
	size_t end = dma_desc->length / sizeof(dma_elem_t) / 8;
	for (size_t i = 0; i < end; ++i) {
		// manually unrolling 4 iterations of the loop here
		dst[0] = src[0].sample1;
		dst[1] = src[2].sample1;
		dst[2] = src[4].sample1;
		dst[3] = src[6].sample1;
		src += 8;
		dst += 4;
	}
	// the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
	if ((dma_desc->length & 0x7) != 0) {
		dst[0] = src[0].sample1;
		dst[1] = src[2].sample1;
	}
}

static void IRAM_ATTR dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc,
		uint8_t* dst) {
	assert(
			s_state->sampling_mode == SM_0A0B_0B0C
					|| s_state->sampling_mode == SM_0A00_0B00);
	size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
	// manually unrolling 4 iterations of the loop here
	for (size_t i = 0; i < end; ++i) {
		dst[0] = src[0].sample1;
		dst[1] = src[1].sample1;
		dst[2] = src[2].sample1;
		dst[3] = src[3].sample1;
		src += 4;
		dst += 4;
	}
	// the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
	if ((dma_desc->length & 0x7) != 0) {
		dst[0] = src[0].sample1;
		dst[1] = src[1].sample1;
		dst[2] = src[2].sample1;
		dst[3] = src[2].sample2;
	}
}

static inline void rgb565_to_888(uint8_t in1, uint8_t in2, uint8_t* dst) {
	dst[0] = (in2 & 0b00011111) << 3; // blue
	dst[1] = ((in1 & 0b111) << 5) | ((in2 & 0b11100000 >> 5)); // green
	dst[2] = in1 & 0b11111000; // red
}

static void IRAM_ATTR dma_filter_rgb565(const dma_elem_t* src,
		lldesc_t* dma_desc, uint8_t* dst) {
	assert(
			s_state->sampling_mode == SM_0A0B_0B0C
					|| s_state->sampling_mode == SM_0A00_0B00);

	const int unroll = 2;         // manually unrolling 2 iterations of the loop
	const int samples_per_pixel = 2;
	const int bytes_per_pixel = 3;
	size_t end = dma_desc->length / sizeof(dma_elem_t) / unroll
			/ samples_per_pixel;
	for (size_t i = 0; i < end; ++i) {
		rgb565_to_888(src[0].sample1, src[1].sample1, &dst[0]);
		rgb565_to_888(src[2].sample1, src[3].sample1, &dst[3]);
		dst += bytes_per_pixel * unroll;
		src += samples_per_pixel * unroll;
	}
	if ((dma_desc->length & 0x7) != 0) {
		rgb565_to_888(src[0].sample1, src[1].sample1, &dst[0]);
		rgb565_to_888(src[2].sample1, src[2].sample2, &dst[3]);
	}
}
