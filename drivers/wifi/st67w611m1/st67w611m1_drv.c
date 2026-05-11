/*
 * Copyright (c) 2019 Tobias Svehagen
 * Copyright (c) 2020 Grinn
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st67w611m1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_st67w611m1, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#include <modem_context.h>
#include <modem_cmd_handler.h>

#include "st67w611m1_drv.h"
#include "st67w611m1_spi_poc.h" // TODO POC

#define ST67W611M1_ETH_MTU 1500

#define BUF_POOL_BUF_COUNT 8
#define BUF_POOL_BUF_SIZE  256

NET_BUF_POOL_DEFINE(mdm_recv_pool, BUF_POOL_BUF_COUNT, BUF_POOL_BUF_SIZE, 0, NULL);

K_KERNEL_STACK_DEFINE(st67_workq_stack, CONFIG_ST67W611M1_WORKQ_STACK_SIZE);

static struct st67_data driver_data;

static inline int st67_send_at_cmd(struct st67_data *data, const struct modem_cmd *cmds,
				   size_t cmds_len, const uint8_t *buf, k_timeout_t timeout)
{
	return modem_cmd_send(&data->mctx.iface, &data->mctx.cmd_handler, cmds, cmds_len, buf,
			      &data->sem_cmd_response_wait, timeout);
}

static inline int st67_write_spi(struct modem_iface *iface, const uint8_t *buf, size_t size)
{
	LOG_DBG("SPI WRITE CALL");
	printk(buf);
	printk("\n");
	return 0;
}

static void st67_scan_work(struct k_work *work)
{
	int ret;
	struct st67_data *data;
	data = CONTAINER_OF(work, struct st67_data, scan_work);
	LOG_DBG("st67_scan_work: started");

	static const struct modem_cmd cmds[] = {
		// MODEM_CMD_DIRECT("+CWLAP:", ); // TODO no direct cmd
		// TODO add scan_done
	};

	ret = st67_send_at_cmd(data, cmds, ARRAY_SIZE(cmds), "AT+CWLAP", ST67W611M1_SCAN_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Scan failed: err %d", ret);
	}

out:
	data->scan_cb = NULL;
	LOG_DBG("scan work end");
}

static int st67_scan(const struct device *dev, struct net_if *iface,
		     struct wifi_scan_params *params, scan_result_cb_t cb)
{
	struct st67_data *data = dev->data;

	ARG_UNUSED(params);

	/* Block wifi scan overlapping */
	if (data->scan_cb != NULL) {
		return -EINPROGRESS;
	}

	if (!net_if_is_carrier_ok(iface)) {
		return -EIO;
	}

	data->scan_cb = cb;

	LOG_DBG("st67_scan: submit scan work");

	k_work_submit_to_queue(&data->workq, &data->scan_work);
	return 0;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", NULL, 0U, ""),
	MODEM_CMD("ERROR", NULL, 0U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("CW:CONNECTED", NULL, 0U, ""),
};

int st67_init(const struct device *dev)
{
	int ret;
	struct st67_data *data = dev->data;

	spi_protocol_init(); // TODO POC

	k_sem_init(&data->sem_cmd_response_wait, 0, 1);

	/* Implemented with work items just like esp_at driver to return the API calls quickly */
	k_work_init(&data->scan_work, st67_scan_work);
	k_work_queue_start(&data->workq, st67_workq_stack, K_KERNEL_STACK_SIZEOF(st67_workq_stack),
			   CONFIG_ST67W611M1_WORKQ_THREAD_PRIORITY, NULL);

	/* cmd handler */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &data->cmd_match_buf[0],
		.match_buf_len = sizeof(data->cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = K_NO_WAIT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = sizeof(response_cmds),
		.unsol_cmds = unsol_cmds,
		.unsol_cmds_len = sizeof(unsol_cmds),
	};
	ret = modem_cmd_handler_init(&data->mctx.cmd_handler, &data->cmd_handler_data,
				     &cmd_handler_config);
	if (ret < 0) {
		LOG_ERR("modem_cmd_handler_init failed: %d", ret);
		goto out;
	}

	ret = modem_context_register(&data->mctx);
	if (ret < 0) {
		LOG_ERR("modem_context_register failed: %d", ret);
		goto out;
	}

	/* cmd handler cb register */
	/* TODO might be implemented in something like drivers/modem/modem_iface_spi.c */
	data->mctx.iface.write = st67_write_spi;

	/* RX thread */

out:
	return ret;
}

static void st67_iface_init(struct net_if *iface)
{
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	ethernet_init(iface);
	net_if_dormant_on(iface); /* Wifi network association not yet done */
}

static const struct wifi_mgmt_ops st67_mgmt_ops = {.scan = st67_scan};

static const struct net_wifi_mgmt_offload st67_api = {
	.wifi_iface.iface_api.init = st67_iface_init,
	.wifi_mgmt_api = &st67_mgmt_ops,
};

ETH_NET_DEVICE_DT_INST_DEFINE(0, st67_init, NULL, &driver_data, NULL, CONFIG_WIFI_INIT_PRIORITY,
			      &st67_api, ST67W611M1_ETH_MTU);
