/*
 * qr_recoginize.c
 *
 *  Created on: 2017年12月31日
 *      Author: sky
 */
#include <stdio.h>
#include <string.h>
#include "quirc_internal.h"
#include "qr_recoginize.h"
#include "camera.h"
#include "quirc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
static char* TAG="QR";
static const char *data_type_str(int dt) {
	switch (dt) {
	case QUIRC_DATA_TYPE_NUMERIC:
		return "NUMERIC";
	case QUIRC_DATA_TYPE_ALPHA:
		return "ALPHA";
	case QUIRC_DATA_TYPE_BYTE:
		return "BYTE";
	case QUIRC_DATA_TYPE_KANJI:
		return "KANJI";
	}

	return "unknown";
}

void dump_cells(const struct quirc_code *code) {
	int u, v;

	printf("    %d cells, corners:", code->size);
	for (u = 0; u < 4; u++)
		printf(" (%d,%d)", code->corners[u].x, code->corners[u].y);
	printf("\n");

	for (v = 0; v < code->size; v++) {
		printf("    ");
		for (u = 0; u < code->size; u++) {
			int p = v * code->size + u;

			if (code->cell_bitmap[p >> 3] & (1 << (p & 7)))
				printf("[]");
			else
				printf("  ");
		}
		printf("\n");
	}
}

void dump_data(const struct quirc_data *data) {
	printf("    Version: %d\n", data->version);
	printf("    ECC level: %c\n", "MLHQ"[data->ecc_level]);
	printf("    Mask: %d\n", data->mask);
	printf("    Data type: %d (%s)\n", data->data_type,
			data_type_str(data->data_type));
	printf("    Length: %d\n", data->payload_len);
	printf("    Payload: %s\n", data->payload);

	if (data->eci)
		printf("    ECI: %d\n", data->eci);
}

static void dump_info(struct quirc *q) {
	int count = quirc_count(q);
	int i;

	printf("%d QR-codes found:\n\n", count);
	for (i = 0; i < count; i++) {
		struct quirc_code code;
		struct quirc_data data;
		quirc_decode_error_t err;

		quirc_extract(q, i, &code);
		err = quirc_decode(&code, &data);

		dump_cells(&code);
		printf("\n");

		if (err) {
			printf("  Decoding FAILED: %s\n", quirc_strerror(err));
		} else {
			printf("  Decoding successful:\n");
			dump_data(&data);
		}

		printf("\n");
	}
}
void qr_recoginze(void *pdata) {
//	i++;
	camera_config_t *camera_config =pdata;
	if((pdata==NULL)||(camera_config->frame_size>CAMERA_FS_VGA))
	{
		ESP_LOGI(TAG,"Camera Size err");
		vTaskDelete(NULL) ;
	}
	printf("begin to qr_recoginze\r\n");
	struct quirc *q;
	struct quirc_data qd;
	uint8_t *image;
	q = quirc_new();
	if (!q) {
		printf("can't create quirc object\r\n");
		vTaskDelete(NULL) ;
	}
	printf("begin to quirc_resize\r\n");
	if (quirc_resize(q, camera_get_fb_width(), camera_get_fb_height()) < 0)
	{
		printf("quirc_resize err\r\n");
		quirc_destroy(q);
		vTaskDelete(NULL) ;
	}image = quirc_begin(q, NULL, NULL);
	memcpy(image, camera_get_fb(), camera_get_data_size());
	quirc_end(q);
	printf("quirc_end\r\n");
	int id_count = quirc_count(q);
	if (id_count == 0) {
		fprintf(stderr, "Error: not a valid qrcode\n");
		quirc_destroy(q);
		vTaskDelete(NULL) ;
	}

	struct quirc_code code;
	quirc_extract(q, 0, &code);
	quirc_decode(&code, &qd);
	dump_info(q);
	quirc_destroy(q);
//	j++;
	printf("finish recoginize\r\n");
	vTaskDelete(NULL) ;
}

//int qr_recoginze() {
//	printf("begin to qr_recoginze\r\n");
//	struct quirc *q;
//	struct quirc_data qd;
//	uint8_t *image;
//	q = quirc_new();
//	if (!q) {
//		perror("can't create quirc object\r\n");
//		return RECONGIZE_FAIL;
//	}
//	if (quirc_resize(q, camera_get_fb_width(), camera_get_fb_height()) < 0)
//		return RECONGIZE_FAIL;
//	image = quirc_begin(q, NULL, NULL);
//	memcpy(image, camera_get_fb(), camera_get_data_size());
//	quirc_end(q);
//	int id_count = quirc_count(q);
//	if (id_count == 0) {
//		fprintf(stderr, "Error: not a valid qrcode\n");
//		return RECONGIZE_FAIL;
//	}
//
//	struct quirc_code code;
//	quirc_extract(q, 0, &code);
//	quirc_decode(&code, &qd);
//	dump_info(q);
//	quirc_destroy(q);
//	return RECONGIZE_OK;
//}
//
