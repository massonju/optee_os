/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, BayLibre
 *
 * TI OMAP/K3 I2C controller driver for OP-TEE
 */

#ifndef __OMAP_I2C_H__
#define __OMAP_I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include <tee_api_types.h>
#include <types_ext.h>

/* I2C message flags */
#define OMAP_I2C_M_RD		0x0001
#define OMAP_I2C_M_TEN		0x0010
#define OMAP_I2C_M_STOP		0x8000
#define OMAP_I2C_M_IGNORE_NAK	0x1000

/* I2C slave events */
enum omap_i2c_slave_event {
	OMAP_I2C_SLAVE_READ_REQUESTED = 0,
	OMAP_I2C_SLAVE_WRITE_REQUESTED,
	OMAP_I2C_SLAVE_READ_PROCESSED,
	OMAP_I2C_SLAVE_WRITE_RECEIVED,
	OMAP_I2C_SLAVE_STOP,
};

/* IP versions */
#define OMAP_I2C_IP_VERSION_1	1
#define OMAP_I2C_IP_VERSION_2	2

/* I2C message descriptor */
struct omap_i2c_msg {
	uint16_t addr;
	uint16_t flags;
	uint16_t len;
	uint8_t *buf;
};

/*
 * Slave callback type.
 * @event: slave event type
 * @val: pointer to data byte
 * @data: opaque user data
 */
typedef TEE_Result (*omap_i2c_slave_cb_t)(enum omap_i2c_slave_event event,
					  uint8_t *val, void *data);

/* Platform configuration */
struct omap_i2c_platform_data {
	uint32_t flags;
	uint32_t ip_rev;	/* OMAP_I2C_IP_VERSION_x */
	uint32_t clkrate;	/* bus speed in kHz */
	uint32_t fclk_rate;	/* functional clock in Hz, 0 = use default */

	/*
	 * Inter-message delay in microseconds for repeated start sequences.
	 *
	 * Hardware I2C slaves (EEPROMs, sensors, etc.) respond within the
	 * I2C clock cycle and do not need any delay between messages (set 0).
	 *
	 * Software-emulated I2C slaves (e.g. Linux kernel slave-24c02 running
	 * on another board) process I2C events via interrupt handlers and
	 * kernel code. They may need significant delay (100-200us) between
	 * the register address write and the subsequent repeated-start read
	 * to update their internal state before responding.
	 */
	uint32_t inter_msg_delay_us;
};

/* Opaque device handle */
struct omap_i2c_dev;

/*
 * Initialize an I2C controller.
 * @base_pa: physical address of register region
 * @base_sz: size of register region
 * @pdata: platform configuration
 * @dev: [out] allocated device handle
 */
TEE_Result omap_i2c_init(paddr_t base_pa, size_t base_sz,
			 const struct omap_i2c_platform_data *pdata,
			 struct omap_i2c_dev **dev);

/*
 * Master-mode transfer of one or more messages (polling).
 * @dev: device handle
 * @msgs: array of messages
 * @num: number of messages
 */
TEE_Result omap_i2c_master_xfer(struct omap_i2c_dev *dev,
				struct omap_i2c_msg *msgs, size_t num);

/*
 * Register as I2C slave.
 * @dev: device handle
 * @addr: slave address
 * @is_10bit: true for 10-bit addressing
 * @cb: callback for slave events
 * @cb_data: opaque data for callback
 */
TEE_Result omap_i2c_slave_register(struct omap_i2c_dev *dev, uint16_t addr,
				   bool is_10bit, omap_i2c_slave_cb_t cb,
				   void *cb_data);

/*
 * Unregister slave listener.
 * @dev: device handle
 */
TEE_Result omap_i2c_slave_unregister(struct omap_i2c_dev *dev);

/*
 * Cleanup and free device.
 * @dev: device handle
 */
void omap_i2c_cleanup(struct omap_i2c_dev *dev);

/*
 * Poll for slave events (blocking, with timeout).
 * Use this when IRQ-based slave is not available.
 *
 * @dev: device handle
 * @timeout_ms: how long to poll (0 = poll once)
 *
 * Returns TEE_SUCCESS if events were handled, TEE_ERROR_NO_DATA if nothing.
 */
TEE_Result omap_i2c_slave_poll(struct omap_i2c_dev *dev, uint32_t timeout_ms);

/*
 * omap_i2c_is_slave_active - Check if the controller is in slave mode
 * @dev: I2C device handle
 *
 * Returns true if the controller is currently registered as an I2C slave.
 */
bool omap_i2c_is_slave_active(struct omap_i2c_dev *dev);

/*
 * omap_i2c_get_slave_addr - Get the configured slave address
 * @dev: I2C device handle
 *
 * Returns the own address set by omap_i2c_slave_register(), or 0 if
 * the controller is not in slave mode.
 */
uint16_t omap_i2c_get_slave_addr(struct omap_i2c_dev *dev);

/*
 * omap_i2c_get_speed_khz - Get the configured bus speed
 * @dev: I2C device handle
 *
 * Returns the I2C bus clock speed in kHz, or 0 if @dev is NULL.
 */
uint32_t omap_i2c_get_speed_khz(struct omap_i2c_dev *dev);

/*
 * omap_i2c_get_inter_msg_delay_us - Get the inter-message delay
 * @dev: I2C device handle
 *
 * Returns the delay in microseconds inserted between messages in a
 * multi-message transfer, or 0 if none is configured.
 */
uint32_t omap_i2c_get_inter_msg_delay_us(struct omap_i2c_dev *dev);

/*
 * omap_i2c_get_fifo_size - Get the hardware FIFO depth
 * @dev: I2C device handle
 *
 * Returns the TX/RX FIFO size in bytes, or 0 if @dev is NULL.
 */
uint16_t omap_i2c_get_fifo_size(struct omap_i2c_dev *dev);

/*
 * omap_i2c_is_bus_busy - Check if the I2C bus is currently busy
 * @dev: I2C device handle
 *
 * Reads the BB (Bus Busy) bit from the status register.
 * Returns true if the bus is busy, false otherwise or if @dev is NULL.
 */
bool omap_i2c_is_bus_busy(struct omap_i2c_dev *dev);

#endif /* __OMAP_I2C_H__ */
