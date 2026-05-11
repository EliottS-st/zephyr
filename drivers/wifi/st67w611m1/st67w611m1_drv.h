/*
 * Copyright (c) 2019 Tobias Svehagen
 * Copyright (c) 2020 Grinn
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_ST67W611M1_DRV_H_
#define ZEPHYR_DRIVERS_WIFI_ST67W611M1_DRV_H_

#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>

#include <modem_context.h>
#include <modem_cmd_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ST67W611M1_SCAN_TIMEOUT K_SECONDS(10)

#define ST67W611M1_CMD_MATCH_BUF_SIZE 256

struct st67_data {
	/* modem commands */
	struct modem_context mctx;
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[ST67W611M1_CMD_MATCH_BUF_SIZE];

	/* wifi */
	scan_result_cb_t scan_cb;

	/* work queue and items */
	struct k_work_q workq;
	struct k_work scan_work;

	/* semaphores */
	struct k_sem sem_cmd_response_wait;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_WIFI_ST67W611M1_DRV_H_ */
