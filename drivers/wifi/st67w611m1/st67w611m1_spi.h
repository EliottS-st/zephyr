/*
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_ST67W611M1_SPI_H_
#define ZEPHYR_DRIVERS_WIFI_ST67W611M1_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

enum st67_spi_frame_type {
	at_cmd = 0x00,
	sta_data = 0x01,
	ap_data = 0x02,
};

void spi_wait_for_rx(void);

int st67_spi_send(const uint8_t *buf, size_t len);

int st67_spi_receive(uint8_t *buf, size_t len);

int st67_spi_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_WIFI_ST67W611M1_SPI_H_ */
