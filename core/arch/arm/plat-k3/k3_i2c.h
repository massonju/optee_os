/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, BayLibre
 *
 * K3 AM64x I2C platform accessors
 */


#ifndef __K3_I2C_H__
#define __K3_I2C_H__

#include <stdint.h>
#include <drivers/omap_i2c.h>

#define K3_I2C_MAX_INSTANCES	2

/*
 * Initialize an I2C instance
 * @instance: 0-1
 * @speed_khz: bus speed in kHz (e.g. 100, 400). 0 defaults to 400.
 * @inter_msg_delay_us: delay between messages in a multi-msg xfer
 *
 * Returns TEE_SUCCESS or error.
 */
TEE_Result k3_i2c_init_instance(uint32_t instance, uint32_t speed_khz,
				uint32_t inter_msg_delay_us);

/*
 * Deinitialize an I2C instance, freeing resources.
 * @instance: 0-1
 */
TEE_Result k3_i2c_deinit_instance(uint32_t instance);

/*
 * Get the device handle for an initialized instance (NULL if not init'd).
 */
struct omap_i2c_dev *k3_i2c_get_dev(uint32_t instance);

#endif /* __K3_I2C_H__ */
