// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, BayLibre
 *
 * K3 AM62x I2C platform integration for OP-TEE
 */

#include <assert.h>
#include <drivers/omap_i2c.h>
#include <kernel/spinlock.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <trace.h>
#include <io.h>

#include "k3_i2c.h"

/*
 * AM62x I2C base addresses
 *
 * I2C0: 0x20000000
 * I2C1: 0x20010000
 * I2C2: 0x20020000
 * I2C3: 0x20030000
 */
static const paddr_t am62x_i2c_bases[K3_I2C_MAX_INSTANCES] = {
	0x20000000,
	0x20010000,
	0x20020000,
	0x20030000,
};

#define AM62X_I2C_SIZE		0x100

/* AM62x uses 48 MHz functional clock for I2C */
#define AM62X_I2C_FCLK_RATE	48000000

static struct omap_i2c_dev *k3_i2c_devs[K3_I2C_MAX_INSTANCES];
static unsigned int k3_i2c_lock = SPINLOCK_UNLOCK;

/*
 * AM62x Main PADCONF base
 */
#define AM62X_PADCONF_BASE	0x000f4000
#define AM62X_PADCONF_SIZE	0x200

/*
 * Pad mux value for I2C function:
 *   bit[3:0]  = mux mode (varies per instance)
 *   bit[16]   = 0     -> no pulldown
 *   bit[17]   = 1     -> pullup enable
 *   bit[18]   = 1     -> input enable
 *
 * 0x00060000 = pullup + input enable, OR'd with mux mode
 */
#define PAD_I2C_FLAGS		0x00060000

/*
 * Per-instance pad configuration
 * From AM62x TRM (SPRUJ40) pin mux tables
 *
 * I2C0: SCL = offset 0x1e0 (mux 0), SDA = offset 0x1e4 (mux 0)
 * I2C1: SCL = offset 0x1e8 (mux 0), SDA = offset 0x1ec (mux 0)
 * I2C2: SCL = offset 0x1f0 (mux 0), SDA = offset 0x1f4 (mux 0)
 * I2C3: SCL = offset 0x1d0 (mux 2), SDA = offset 0x1d4 (mux 2)
 */
struct i2c_pad_config {
	uint16_t scl_offset;
	uint16_t sda_offset;
	uint32_t mux_mode;
};

static const struct i2c_pad_config am62x_i2c_pads[K3_I2C_MAX_INSTANCES] = {
	[0] = { .scl_offset = 0x1e0, .sda_offset = 0x1e4, .mux_mode = 0 },
	[1] = { .scl_offset = 0x1e8, .sda_offset = 0x1ec, .mux_mode = 0 },
	[2] = { .scl_offset = 0x1f0, .sda_offset = 0x1f4, .mux_mode = 0 },
	[3] = { .scl_offset = 0x1d0, .sda_offset = 0x1d4, .mux_mode = 2 },
};

static TEE_Result k3_i2c_configure_pads(uint32_t instance)
{
	const struct i2c_pad_config *pad;
	uint32_t val;
	vaddr_t base;

	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	pad = &am62x_i2c_pads[instance];

	base = (vaddr_t)core_mmu_add_mapping(MEM_AREA_IO_SEC,
					     AM62X_PADCONF_BASE,
					     AM62X_PADCONF_SIZE);
	if (!base) {
		EMSG("Failed to map PADCONF registers");
		return TEE_ERROR_GENERIC;
	}

	val = PAD_I2C_FLAGS | pad->mux_mode;

	DMSG("I2C%" PRIu32 " pad config: SCL=0x%03x SDA=0x%03x val=0x%08x",
	     instance, pad->scl_offset, pad->sda_offset, val);

	io_write32(base + pad->scl_offset, val);
	io_write32(base + pad->sda_offset, val);

	return TEE_SUCCESS;
}

TEE_Result k3_i2c_init_instance(uint32_t instance, uint32_t speed_khz,
				uint32_t inter_msg_delay_us)
{
	struct omap_i2c_platform_data pdata = {
		.ip_rev = OMAP_I2C_IP_VERSION_2,
		.clkrate = speed_khz ? speed_khz : 400,
		.fclk_rate = AM62X_I2C_FCLK_RATE,
		.inter_msg_delay_us = inter_msg_delay_us,
		.flags = 0,
	};
	struct omap_i2c_dev *dev;
	TEE_Result res;
	uint32_t exceptions;

	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = cpu_spin_lock_xsave(&k3_i2c_lock);

	/* Already initialized — return success */
	if (k3_i2c_devs[instance]) {
		cpu_spin_unlock_xrestore(&k3_i2c_lock, exceptions);
		DMSG("I2C%" PRIu32 " already initialized", instance);
		return TEE_SUCCESS;
	}

	cpu_spin_unlock_xrestore(&k3_i2c_lock, exceptions);

	/* Configure pads */
	res = k3_i2c_configure_pads(instance);
	if (res) {
		EMSG("Pad config failed for I2C%" PRIu32 ": 0x%x",
		     instance, res);
		return res;
	}

	DMSG("Initializing I2C%" PRIu32 " at PA 0x%lx speed=%" PRIu32 "kHz"
	     " inter_msg_delay=%" PRIu32 "us",
	     instance, (unsigned long)am62x_i2c_bases[instance],
	     pdata.clkrate, inter_msg_delay_us);

	res = omap_i2c_init(am62x_i2c_bases[instance], AM62X_I2C_SIZE,
			    &pdata, &dev);
	if (res) {
		EMSG("Failed to init I2C%" PRIu32 ": 0x%x", instance, res);
		return res;
	}

	exceptions = cpu_spin_lock_xsave(&k3_i2c_lock);
	k3_i2c_devs[instance] = dev;
	cpu_spin_unlock_xrestore(&k3_i2c_lock, exceptions);

	return TEE_SUCCESS;
}

TEE_Result k3_i2c_deinit_instance(uint32_t instance)
{
	struct omap_i2c_dev *dev;
	uint32_t exceptions;

	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = cpu_spin_lock_xsave(&k3_i2c_lock);
	dev = k3_i2c_devs[instance];
	k3_i2c_devs[instance] = NULL;
	cpu_spin_unlock_xrestore(&k3_i2c_lock, exceptions);

	if (dev)
		omap_i2c_cleanup(dev);

	DMSG("I2C%" PRIu32 " deinitialized", instance);

	return TEE_SUCCESS;
}

struct omap_i2c_dev *k3_i2c_get_dev(uint32_t instance)
{
	if (instance >= K3_I2C_MAX_INSTANCES)
		return NULL;

	return k3_i2c_devs[instance];
}
