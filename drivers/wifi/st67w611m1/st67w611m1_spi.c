/*
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st67w611m1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(st67w611m1_spi, CONFIG_WIFI_LOG_LEVEL);

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/toolchain.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/net_buf.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>

#include "st67w611m1_spi.h"

struct st67_config {
	const struct gpio_dt_spec chip_select;
	const struct gpio_dt_spec ready;
	const struct gpio_dt_spec chip_en;
	const struct gpio_dt_spec boot;
	const struct spi_dt_spec spi;
};

#define ST67_SPI_NODE DT_NODELABEL(st67w611m1)

static const struct st67_config st67_config = {
	.chip_select = GPIO_DT_SPEC_GET(ST67_SPI_NODE, chip_select_gpios),
	.ready = GPIO_DT_SPEC_GET(ST67_SPI_NODE, ready_gpios),
	.chip_en = GPIO_DT_SPEC_GET(ST67_SPI_NODE, chip_en_gpios),
	.boot = GPIO_DT_SPEC_GET(ST67_SPI_NODE, boot_gpios),
	.spi = SPI_DT_SPEC_GET(ST67_SPI_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_LOCK_ON),
};

#define ST67_SPI_SYNC_WORD 0x55AA

struct st67_spi_context {
	bool initialized;
	bool ready_pin_previous_state;

	struct k_event event;
	struct k_fifo tx_fifo;
	struct k_mutex spi_send_mutex;

	drv_rx_iface_cb_t drv_rx_iface_cb[FRAME_TYPE_MAX];

	uint8_t tx_temp_buf[ST67_SPI_BUFFER_SIZE_BYTES]; // TODO will be removed
};

static struct st67_spi_context spi_context = {
	.initialized = false,
	.ready_pin_previous_state = false,
};

K_KERNEL_STACK_DEFINE(st67_spi_processor_stack, CONFIG_ST67W611M1_SPI_PROCESSOR_THREAD_STACK_SIZE);

static struct k_thread st67_spi_processor_thread;

enum {
	SPI_EVT_ST67_READY = BIT(1), /* Ready pin rising edge.
	ST67 has something to send AND/OR is ready to receive. */

	SPI_EVT_ST67_ACK = BIT(2), /* Ready pin falling edge.
	ST67 acks what it received. */

	SPI_EVT_TX_PENDING = BIT(3),
};

#define ST67_SPI_HEADER_VERSION_SHIFT 0
#define ST67_SPI_HEADER_VERSION_MASK  BIT_MASK(2)
#define ST67_SPI_HEADER_VERSION_GET(x)                                                             \
	(((x) >> ST67_SPI_HEADER_VERSION_SHIFT) & ST67_SPI_HEADER_VERSION_MASK)

#define ST67_SPI_HEADER_RX_STALL_SHIFT 2
#define ST67_SPI_HEADER_RX_STALL_MASK  BIT_MASK(1)
#define ST67_SPI_HEADER_RX_STALL_GET(x)                                                            \
	(((x) >> ST67_SPI_HEADER_RX_STALL_SHIFT) & ST67_SPI_HEADER_RX_STALL_MASK)

#define ST67_SPI_HEADER_FLAGS_SHIFT 3
#define ST67_SPI_HEADER_FLAGS_MASK  BIT_MASK(5)
#define ST67_SPI_HEADER_FLAGS_GET(x)                                                               \
	(((x) >> ST67_SPI_HEADER_FLAGS_SHIFT) & ST67_SPI_HEADER_FLAGS_MASK)

static uint8_t spi_buffer[ST67_SPI_BUFFER_SIZE_BYTES] __nocache;

RING_BUF_DECLARE(tx_ring_buf, ST67_SPI_BUFFER_SIZE_BYTES *CONFIG_ST67W611M1_TX_BUF_COUNT);
K_MEM_SLAB_DEFINE(tx_fifo_item_slab, sizeof(struct fifo_item), 50, 4);

#define ST67W611M1_SEND_MUTEX_WAIT K_SECONDS(5)

int st67_spi_send_bytes(const uint8_t *buf, size_t len)
{
	int ret;

	ret = k_mutex_lock(&spi_context.spi_send_mutex, ST67W611M1_SEND_MUTEX_WAIT);
	if (ret < 0) {
		return ret;
	}

	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	if (!spi_context.initialized) {
		LOG_ERR("SPI not initialized");
		ret = -EINVAL;
		goto out;
	}

	if (len > ring_buf_space_get(&tx_ring_buf)) {
		ret = -ENOMEM;
		goto out;
	}

	struct fifo_item *fifo_item;
	ret = k_mem_slab_alloc(&tx_fifo_item_slab, (void **)&fifo_item,
			       K_NO_WAIT); /* Freed at the end of st67_spi_process() */
	if (ret < 0) {
		LOG_ERR("tx fifo slab full");
		goto out;
	}

	ring_buf_put(&tx_ring_buf, buf, len);

	fifo_item->len = len;
	fifo_item->type = AT_CMD;

	k_fifo_put(&spi_context.tx_fifo, fifo_item);
	k_event_post(&spi_context.event, SPI_EVT_TX_PENDING);

out:
	k_mutex_unlock(&spi_context.spi_send_mutex);
	return ret;
}

int st67_spi_send_net_pkt(struct net_pkt *pkt)
{
	int ret;

	ret = k_mutex_lock(&spi_context.spi_send_mutex, ST67W611M1_SEND_MUTEX_WAIT);
	if (ret < 0) {
		return ret;
	}

	if (!pkt) {
		ret = -EINVAL;
		goto out;
	}

	if (!spi_context.initialized) {
		LOG_ERR("SPI not initialized");
		ret = -EINVAL;
		goto out;
	}

	size_t pkt_len = net_pkt_get_len(pkt);

	if (pkt_len > ring_buf_space_get(&tx_ring_buf)) {
		ret = -ENOMEM;
		goto out;
	}

	struct fifo_item *fifo_item;
	ret = k_mem_slab_alloc(&tx_fifo_item_slab, (void **)&fifo_item,
			       K_NO_WAIT); /* Freed at the end of st67_spi_process() */
	if (ret < 0) {
		LOG_ERR("tx fifo slab full");
		goto out;
	}

	size_t copied = net_buf_linearize(spi_context.tx_temp_buf, sizeof(spi_context.tx_temp_buf),
					  pkt->buffer, 0, pkt_len);
	if (copied != pkt_len) {
		ret = -EIO;
		goto out;
	}

	ring_buf_put(&tx_ring_buf, spi_context.tx_temp_buf, pkt_len);

	fifo_item->len = pkt_len;
	fifo_item->type = STA_DATA;

	k_fifo_put(&spi_context.tx_fifo, fifo_item);
	k_event_post(&spi_context.event, SPI_EVT_TX_PENDING);

out:
	k_mutex_unlock(&spi_context.spi_send_mutex);
	return ret;
}

static void write_spi_header_to_buffer(size_t len, uint8_t type)
{
	sys_put_le16(ST67_SPI_SYNC_WORD, &spi_buffer[0]);
	sys_put_le16((uint16_t)len, &spi_buffer[2]);
	spi_buffer[4] = 0; /* frame */
	spi_buffer[5] = type;
	sys_put_le16(0, &spi_buffer[6]); /* reserved */
}

struct rx_header {
	uint16_t data_length;
	uint8_t type;
	bool rx_stall;
};

static int parse_spi_header(struct rx_header *out)
{
	if (!out) {
		return -EINVAL;
	}

	uint16_t sync_word = sys_get_le16(&spi_buffer[0]);
	if (sync_word != ST67_SPI_SYNC_WORD) {
		return -EBADMSG;
	}

	out->data_length = sys_get_le16(&spi_buffer[2]);
	out->rx_stall = ST67_SPI_HEADER_RX_STALL_GET(spi_buffer[4]);
	out->type = spi_buffer[5];

	return 0;
}

static int st67_spi_txrx(size_t xfer_len, size_t offset)
{
	struct spi_buf tx_buf = {
		.buf = &spi_buffer[offset],
		.len = xfer_len,
	};
	struct spi_buf rx_buf = {
		.buf = &spi_buffer[offset],
		.len = xfer_len,
	};
	struct spi_buf_set tx_buf_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx_buf_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	return spi_transceive_dt(&st67_config.spi, &tx_buf_set, &rx_buf_set);
}

#define ST67_SPI_BUF_ALIGN_MASK 0x3

static int st67_spi_process()
{
	int ret = 0;
	size_t pkt_len = 0;
	size_t first_xfer_len = 0;

	struct fifo_item *tx_fifo_item = k_fifo_get(&spi_context.tx_fifo, K_NO_WAIT);

	if (!tx_fifo_item) { /* Nothing to say */
		write_spi_header_to_buffer(0, 0);
	} else {
		pkt_len = tx_fifo_item->len;
		if (pkt_len + sizeof(struct spi_header) > ST67_SPI_BUFFER_SIZE_BYTES) {
			ret = -EMSGSIZE;
			goto out;
		}
		write_spi_header_to_buffer(pkt_len, tx_fifo_item->type);
		ring_buf_get(&tx_ring_buf, spi_buffer + sizeof(struct spi_header), pkt_len);
	}

	/* Add mod4 payload padding to transfered length. */
	first_xfer_len = (pkt_len + sizeof(struct spi_header) + ST67_SPI_BUF_ALIGN_MASK) &
			 ~ST67_SPI_BUF_ALIGN_MASK;

	/* Actual SPI transfer */
	ret = st67_spi_txrx(first_xfer_len, 0);
	if (ret < 0) {
		goto out;
	}

	struct rx_header rx_header;
	ret = parse_spi_header(&rx_header);
	if (ret < 0) {
		goto out;
	}

	/* TODO handle rx stall */

	/* 2nd transaction can be necessary if RX len > TX len. */
	size_t second_xfer_len = 0;
	size_t rx_data_length = (size_t)rx_header.data_length;
	enum st67_spi_frame_type rx_type = rx_header.type;
	size_t first_payload_len = first_xfer_len - sizeof(struct spi_header);

	if (rx_data_length > first_payload_len) {
		second_xfer_len = (rx_data_length - first_payload_len + ST67_SPI_BUF_ALIGN_MASK) &
				  ~ST67_SPI_BUF_ALIGN_MASK;
		if (first_xfer_len + second_xfer_len > ST67_SPI_BUFFER_SIZE_BYTES) {
			ret = -EMSGSIZE;
			goto out;
		}
		ret = st67_spi_txrx(second_xfer_len, first_xfer_len);
		if (ret < 0) {
			goto out;
		}
	}

	if (rx_data_length > 0) {
		if (rx_type >= FRAME_TYPE_MAX) {
			LOG_DBG("Unexpected frame type %d", rx_type);
			goto out;
		}
		if (!spi_context.drv_rx_iface_cb[rx_type]) {
			LOG_DBG("NULL cb");
			goto out;
		}

		spi_context.drv_rx_iface_cb[rx_type](spi_buffer + sizeof(struct spi_header),
						     rx_data_length);
	}

out:
	spi_release_dt(&st67_config.spi);
	if (tx_fifo_item) {
		k_mem_slab_free(&tx_fifo_item_slab, tx_fifo_item);
	}
	return ret;
}

/* Processing SPI on its own thread */
static void st67_spi_processor(void *p1, void *p2, void *p3)
{
	int ret;
	uint32_t received_event;
	while (true) {
		received_event = k_event_wait_safe(&spi_context.event,
						   SPI_EVT_ST67_READY | SPI_EVT_TX_PENDING, false,
						   K_FOREVER);

		ret = gpio_pin_set_dt(&st67_config.chip_select, 1);
		if (ret < 0) {
			LOG_ERR("Cannot set CS");
		}

		/* If ST67 does not have anything to transfer */
		if (!(received_event & SPI_EVT_ST67_READY)) {
			k_event_wait_safe(&spi_context.event, SPI_EVT_ST67_READY, false, K_FOREVER);
		}

		st67_spi_process();

		k_event_wait_safe(&spi_context.event, SPI_EVT_ST67_ACK, false, K_FOREVER);

		ret = gpio_pin_set_dt(&st67_config.chip_select, 0);
		if (ret < 0) {
			LOG_ERR("Cannot unset CS");
		}
	}
}

static void spi_ready_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	LOG_DBG("Got ready");
	if (!spi_context.initialized) {
		LOG_DBG("ready dropped because st67 not initialized");
		return;
	}
	bool pin_current_state = gpio_pin_get_dt(&st67_config.ready);
	if (pin_current_state != spi_context.ready_pin_previous_state) {
		if (pin_current_state) {
			k_event_post(&spi_context.event, SPI_EVT_ST67_READY);
		} else {
			k_event_post(&spi_context.event, SPI_EVT_ST67_ACK);
		}
	} else {
		LOG_DBG("reached current_pin == previous");
	}
	spi_context.ready_pin_previous_state = pin_current_state;
}

int st67_spi_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&st67_config.chip_select) || !gpio_is_ready_dt(&st67_config.ready) ||
	    !gpio_is_ready_dt(&st67_config.chip_en) || !gpio_is_ready_dt(&st67_config.boot) ||
	    !spi_is_ready_dt(&st67_config.spi)) {
		ret = -ENODEV;
		goto out;
	}

	ret = gpio_pin_configure_dt(&st67_config.ready, GPIO_INPUT);
	if (ret < 0) {
		goto out;
	}
	ret = gpio_pin_interrupt_configure_dt(&st67_config.ready, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		goto out;
	}
	static struct gpio_callback cb;
	gpio_init_callback(&cb, spi_ready_cb, BIT(st67_config.ready.pin));
	ret = gpio_add_callback_dt(&st67_config.ready, &cb);
	if (ret < 0) {
		goto out;
	}

	ret = gpio_pin_configure_dt(&st67_config.chip_select, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		goto out;
	}
	ret = gpio_pin_configure_dt(&st67_config.chip_en, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		goto out;
	}
	/* Setting boot to 0 configures the ST67W611M1 to use the firmware on its flash */
	ret = gpio_pin_configure_dt(&st67_config.boot, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		goto out;
	}

	/* event, fifos, sem etc. */
	k_event_init(&spi_context.event);
	k_fifo_init(&spi_context.tx_fifo);
	k_mutex_init(&spi_context.spi_send_mutex);

	/* SPI processing thread */
	k_thread_create(&st67_spi_processor_thread, st67_spi_processor_stack,
			K_THREAD_STACK_SIZEOF(st67_spi_processor_stack), st67_spi_processor, NULL,
			NULL, NULL, K_PRIO_COOP(CONFIG_ST67W611M1_SPI_PROCESSOR_THREAD_PRIORITY), 0,
			K_NO_WAIT);
	k_thread_name_set(&st67_spi_processor_thread, "st67_spi_process_thread");

	/* Give some time to ST67 before power on. */
	k_sleep(K_MSEC(1));

	/* Start ST67W611M1. */
	ret = gpio_pin_set_dt(&st67_config.chip_en, 1);
	if (ret < 0) {
		goto out;
	}

	spi_context.initialized = true;

out:
	return ret;
}

void st67_spi_register_drv_rx_iface_cb(drv_rx_iface_cb_t cb, enum st67_spi_frame_type type)
{
	if (type >= FRAME_TYPE_MAX) {
		LOG_DBG("Unexpected frame type %d", type);
		return;
	}

	spi_context.drv_rx_iface_cb[type] = cb;
}
