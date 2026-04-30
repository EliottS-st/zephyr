/*
 * Copyright (c) 2019 Tobias Svehagen
 * Copyright (c) 2020 Grinn
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "modem_context.h"
#include "zephyr/sys/printk.h" // to be removed
#include <stdint.h>
#define DT_DRV_COMPAT st_st67w611m1

#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#include <modem_cmd_handler.h>

#include "st67w611m1_drv.h"
#include "st67w611m1_spi_poc.h" // TODO POC

LOG_MODULE_REGISTER(wifi_st67w611m1, CONFIG_WIFI_LOG_LEVEL);

#define ST67W611M1_ETH_MTU 1500

K_KERNEL_STACK_DEFINE(st67_workq_stack, CONFIG_ST67W611M1_WORKQ_STACK_SIZE)

static struct st67_data driver_data;

static inline int st67_send_at_cmd(struct st67_data *data, const struct modem_cmd *cmds,
				   size_t cmds_len, const uint8_t *buf, k_timeout_t timeout)
{
	return modem_cmd_send(&data->mctx.iface, &data->mctx.cmd_handler, cmds, cmds_len, buf,
			      &data->sem_cmd_response_wait, timeout);
}

static inline int st67_write_spi(struct modem_iface *iface, const uint8_t *buf, size_t size) {
	printk("SPI WRITE CALL\n");
	return 0;

}

static void st67_scan_work(struct k_work *work)
{
	int ret;
	struct st67_data *data;
	data = CONTAINER_OF(work, struct st67_data, scan_work);
	LOG_INF("st67_scan_work: started");

	static const struct modem_cmd cmds[] = {
		// MODEM_CMD_DIRECT("+CWLAP:", ); // TODO no direct cmd
		// TODO add scan_done
	};

	ret = st67_send_at_cmd(data, cmds, ARRAY_SIZE(cmds), "AT+CWLAP", ST67W611M1_SCAN_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Scan failed: err %d", ret);
	}
	printk("This is never reached because of the lack of scan done in the array\n");
}

static int st67_scan(const struct device *dev, struct wifi_scan_params *params, scan_result_cb_t cb)
{
	struct st67_data *data = dev->data;

	ARG_UNUSED(params);
	ARG_UNUSED(cb);

	LOG_INF("st67_scan: submit scan work");

	k_work_submit_to_queue(&data->workq, &data->scan_work);
	return 0;
}

int st67_init(const struct device *dev)
{
	int ret;
	struct st67_data *data = dev->data;

	// spi_protocol_init(); // TODO POC

	/* Implemented with work items just like esp_at driver to return the API calls quickly */
	k_work_init(&data->scan_work, st67_scan_work);
	k_work_queue_start(&data->workq, st67_workq_stack, K_KERNEL_STACK_SIZEOF(st67_workq_stack),
			   CONFIG_ST67W611M1_WORKQ_THREAD_PRIORITY, NULL);

	/* cmd handler */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.alloc_timeout = K_NO_WAIT,
		.eol = "\r\n",
		.user_data = NULL,
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
