/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, BayLibre
 *
 * PTA for K3 I2C access
 */

#ifndef PTA_K3_I2C_H
#define PTA_K3_I2C_H

#define PTA_K3_I2C_UUID \
	{ 0xa6b1e07e, 0x4a1e, 0x4b5d, { \
		0x8c, 0x3a, 0x2b, 0x9f, 0x5e, 0x6d, 0x7c, 0x8a } }

/*
 * CMD 0: Ping — verify PTA is reachable
 *   [0] value input:  a = val_a, b = val_b
 *   [1] value output: a = val_a + 1, b = val_b + 2
 */
#define PTA_K3_I2C_CMD_PING			0

/*
 * CMD 1: Get instance status
 *   [0] value.a = instance
 *   [1] value output:
 *	 a = packed status:
 *	     bits [1:0]	 = mode (PTA_K3_I2C_MODE_*)
 *	     bit  [2]	 = bus busy
 *	     bits [15:8] = FIFO size in bytes
 *	 b = speed in kHz
 *   [2] value output:
 *	 a = slave address (0 if not in slave mode)
 *	 b = inter-message delay in us
 *   [3] value output:
 *	 a = slave write count
 *	 b = slave read count
 */
#define PTA_K3_I2C_CMD_STATUS			1

/*
 * CMD 2: Initialize an I2C instance
 *   [0] value.a = instance (0-3), value.b = speed_khz (0 = default 400)
 *   [1] value.a = inter_msg_delay_us
 */
#define PTA_K3_I2C_CMD_INIT			2

/*
 * CMD 3: Deinitialize an I2C instance
 *   [0] value.a = instance
 */
#define PTA_K3_I2C_CMD_DEINIT			3

/*
 * CMD 4: Simple read (value params only, up to 4 bytes)
 *   [0] value.a = instance, value.b = slave address
 *   [1] value.a = register address, value.b = read length (1-4)
 *   [2] value output: a = bytes packed little-endian, b = actual length
 */
#define PTA_K3_I2C_CMD_SIMPLE_READ		4

/*
 * CMD 5: Simple write (value params only, up to 3 data bytes)
 *   [0] value.a = instance, value.b = slave address
 *   [1] value.a = register address
 *   [2] value.a = data bytes packed little-endian, value.b = length (1-3)
 */
#define PTA_K3_I2C_CMD_SIMPLE_WRITE		5

/*
 * CMD 6: Register as I2C slave
 *   [0] value.a = instance, value.b = slave address
 */
#define PTA_K3_I2C_CMD_SLAVE_REGISTER		6

/*
 * CMD 7: Unregister I2C slave
 *   [0] value.a = instance
 */
#define PTA_K3_I2C_CMD_SLAVE_UNREGISTER		7

/*
 * CMD 8: Set slave register contents
 *   [0] value.a = instance
 *   [1] value.a = start register offset, value.b = length
 *   [2] value.a = data bytes 0-3 packed little-endian
 *   [3] value.a = data bytes 4-7 packed little-endian
 */
#define PTA_K3_I2C_CMD_SLAVE_SET_REGS		8

/*
 * CMD 9: Get slave register contents
 *   [0] value.a = instance
 *   [1] value.a = start register offset, value.b = length
 *   [2] value output: a = bytes 0-3 packed little-endian
 *   [3] value output: a = bytes 4-7 packed little-endian
 */
#define PTA_K3_I2C_CMD_SLAVE_GET_REGS		9

/*
 * CMD 10: Poll for slave events
 *   [0] value.a = instance, value.b = timeout_ms
 *   [1] value output: a = write_count, b = read_count
 */
#define PTA_K3_I2C_CMD_SLAVE_POLL		10

#endif /* PTA_K3_I2C_H */
