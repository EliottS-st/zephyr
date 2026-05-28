/*
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st67w611m1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_st67w611m1_spi, CONFIG_WIFI_LOG_LEVEL);

#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

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
	.spi = SPI_DT_SPEC_GET(ST67_SPI_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
};

#define ST67_SPI_SYNC_WORD   0x55AA
#define ST67_SPI_BUFFER_SIZE 1600

struct st67_spi_context {
	bool initialized;
	bool ready_pin_previous_state;
	struct k_event event;
	struct k_queue txq;
	struct k_queue rxq;
	struct k_sem sem_rx_wait;
};

static uint8_t spi_buffer[ST67_SPI_BUFFER_SIZE];

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

struct spi_header {
	uint16_t sync_word;
	uint16_t data_length;
	uint8_t frame; /* version (0:1); rx_stall(2); flags(3:7) */
	uint8_t type;
	uint16_t reserved;
};

BUILD_ASSERT(sizeof(struct spi_header) == 8, "spi_header struct should be 8 bytes long!");

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

void spi_wait_for_rx(void)
{
	k_sem_take(&spi_context.sem_rx_wait, K_FOREVER);
}

struct spi_pkt {
	void *queue_reserved;
	const uint8_t *buf;
	size_t len;
};

int st67_spi_send(const uint8_t *buf, size_t len)
{
	if (!buf) {
		return -EINVAL;
	}

	if (!spi_context.initialized) {
		LOG_ERR("SPI not initialized");
		return -EINVAL;
	}

	struct spi_pkt *pkt =
		k_malloc(sizeof(struct spi_pkt)); /* Freed at the end of st67_spi_process() */
	if (!pkt) {
		return -ENOMEM;
	}
	pkt->buf = buf;
	pkt->len = len;

	k_queue_append(&spi_context.txq, pkt);
	k_event_post(&spi_context.event, SPI_EVT_TX_PENDING);

	return 0;
}

int st67_spi_receive(uint8_t *buf, size_t len)
{
	return 0;
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

static void st67_spi_txrx(size_t xfer_len, size_t offset)
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

	spi_transceive_dt(&st67_config.spi, &tx_buf_set, &rx_buf_set);
}

#define ST67_SPI_BUF_ALIGN_MASK 0x3

static int st67_spi_process()
{
	int ret = 0;
	size_t pkt_len = 0;
	size_t first_xfer_len = 0;
	size_t second_xfer_len = 0;

	/* Prepare TX */
	struct spi_pkt *pkt = k_queue_get(&spi_context.txq, K_NO_WAIT);

	if (!pkt) { /* Nothing to say */
		write_spi_header_to_buffer(0, 0);
	} else {
		pkt_len = pkt->len;
		if (pkt_len + sizeof(struct spi_header) > ST67_SPI_BUFFER_SIZE) {
			ret = -EMSGSIZE;
			goto out;
		}
		write_spi_header_to_buffer(pkt_len, 0);
		memcpy(spi_buffer + sizeof(struct spi_header), pkt->buf, pkt_len);
	}

	first_xfer_len = (pkt_len + sizeof(struct spi_header) + ST67_SPI_BUF_ALIGN_MASK) &
			 ~ST67_SPI_BUF_ALIGN_MASK;

	/* Start sending TX and receive RX header on spi_buffer (for RX len). */
	st67_spi_txrx(first_xfer_len, 0);

	struct rx_header rx_header;
	ret = parse_spi_header(&rx_header);
	if (ret < 0) {
		goto out;
	}

	/* TODO handle rx stall */

	/* Determine remaining len. */
	/* If rx is bigger than tx */
	if ((size_t)rx_header.data_length > first_xfer_len - sizeof(struct spi_header)) {
		second_xfer_len = (size_t)rx_header.data_length;
	} else {
		second_xfer_len = pkt_len - (first_xfer_len - sizeof(struct spi_header));
	}

	if (second_xfer_len > 0) {
		second_xfer_len =
			(second_xfer_len + ST67_SPI_BUF_ALIGN_MASK) & ~ST67_SPI_BUF_ALIGN_MASK;
		st67_spi_txrx(second_xfer_len, first_xfer_len);
	}

out:
	k_free(pkt);
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

		ret = st67_spi_process();
		if (ret < 0) {
			LOG_ERR("got %d", ret);
		}

		k_event_wait_safe(&spi_context.event, SPI_EVT_ST67_ACK, false, K_FOREVER);

		ret = gpio_pin_set_dt(&st67_config.chip_select, 0);
		if (ret < 0) {
			LOG_ERR("Cannot unset CS");
		}

		// k_event_clear(&spi_context.event, ~0); // TODO might do something like this
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

	/* GPIO */
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

	/* event, queues, sem etc. */
	k_event_init(&spi_context.event);
	k_queue_init(&spi_context.txq);
	k_queue_init(&spi_context.rxq);
	k_sem_init(&spi_context.sem_rx_wait, 0, 1);

	/* SPI processing thread */
	k_thread_create(&st67_spi_processor_thread, st67_spi_processor_stack,
			K_THREAD_STACK_SIZEOF(st67_spi_processor_stack), st67_spi_processor, NULL,
			NULL, NULL, K_PRIO_COOP(CONFIG_ST67W611M1_SPI_PROCESSOR_THREAD_PRIORITY), 0,
			K_NO_WAIT);
	k_thread_name_set(&st67_spi_processor_thread, "st67_spi_process_thread");

	/* Start ST67W611M1 */
	ret = gpio_pin_set_dt(&st67_config.chip_en, 1);
	if (ret < 0) {
		goto out;
	}

	spi_context.initialized = true;

out:
	return ret;
}
