// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, BayLibre
 *
 * PTA for K3 I2C access — master and slave (polled) modes
 */

#include <drivers/omap_i2c.h>
#include <kernel/pseudo_ta.h>
#include <kernel/spinlock.h>
#include <mm/core_memprot.h>
#include <string.h>
#include <trace.h>

#include "k3_i2c.h"
#include "pta_k3_i2c.h"

#define PTA_NAME "pta_k3_i2c"

#define PTA_K3_I2C_MODE_UNINITIALIZED  0
#define PTA_K3_I2C_MODE_MASTER	       1
#define PTA_K3_I2C_MODE_SLAVE	       2
#define PTA_K3_I2C_STATUS_BUS_BUSY     BIT(2)
#define PTA_K3_I2C_STATUS_FIFO_SHIFT   8
#define PTA_K3_I2C_STATUS_FIFO_MASK    0xff

#define SLAVE_REG_SIZE	255

struct slave_state {
	uint8_t regs[SLAVE_REG_SIZE];
	uint8_t reg_ptr;
	bool first_byte;
	bool is_read;

	uint32_t write_count;
	uint32_t read_count;

	bool active;

	unsigned int lock;
};

static struct slave_state slave_states[K3_I2C_MAX_INSTANCES];

static TEE_Result slave_callback(enum omap_i2c_slave_event event,
				 uint8_t *val, void *data)
{
	struct slave_state *ss = (struct slave_state *)data;

	switch (event) {
	case OMAP_I2C_SLAVE_WRITE_REQUESTED:
		ss->first_byte = true;
		ss->is_read = false;
		break;

	case OMAP_I2C_SLAVE_WRITE_RECEIVED:
		if (ss->first_byte) {
			ss->reg_ptr = *val;
			ss->first_byte = false;
		} else {
			if (ss->reg_ptr < SLAVE_REG_SIZE)
				ss->regs[ss->reg_ptr] = *val;
			ss->reg_ptr++;
		}
		break;

	case OMAP_I2C_SLAVE_READ_REQUESTED:
		ss->is_read = true;
		if (ss->reg_ptr < SLAVE_REG_SIZE)
			*val = ss->regs[ss->reg_ptr];
		else
			*val = 0xff;
		ss->reg_ptr++;
		break;

	case OMAP_I2C_SLAVE_READ_PROCESSED:
		ss->is_read = true;
		if (ss->reg_ptr < SLAVE_REG_SIZE)
			*val = ss->regs[ss->reg_ptr];
		else
			*val = 0xff;
		ss->reg_ptr++;
		break;

	case OMAP_I2C_SLAVE_STOP:
		if (ss->is_read)
			ss->read_count++;
		else
			ss->write_count++;
		ss->first_byte = true;
		break;

	default:
		break;
	}

	return TEE_SUCCESS;
}

static TEE_Result get_dev(uint32_t instance, struct omap_i2c_dev **dev)
{
	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	*dev = k3_i2c_get_dev(instance);
	if (!*dev)
		return TEE_ERROR_BAD_STATE;

	return TEE_SUCCESS;
}

static TEE_Result cmd_ping(uint32_t param_types,
			   TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	DMSG("PING: a=%" PRIu32 " b=%" PRIu32,
	     params[0].value.a, params[0].value.b);

	params[1].value.a = params[0].value.a + 1;
	params[1].value.b = params[0].value.b + 2;

	return TEE_SUCCESS;
}

static TEE_Result cmd_status(uint32_t param_types,
			     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT);
	struct omap_i2c_dev *dev;
	struct slave_state *ss;
	uint32_t instance;
	uint32_t packed = 0;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	dev = k3_i2c_get_dev(instance);
	ss = &slave_states[instance];

	if (!dev) {
		packed = PTA_K3_I2C_MODE_UNINITIALIZED;
		params[1].value.a = packed;
		params[1].value.b = 0;
		params[2].value.a = 0;
		params[2].value.b = 0;
		params[3].value.a = ss->write_count;
		params[3].value.b = ss->read_count;
		return TEE_SUCCESS;
	}

	packed = omap_i2c_is_slave_active(dev) ? PTA_K3_I2C_MODE_SLAVE :
						 PTA_K3_I2C_MODE_MASTER;
	if (omap_i2c_is_bus_busy(dev))
		packed |= PTA_K3_I2C_STATUS_BUS_BUSY;
	packed |= (omap_i2c_get_fifo_size(dev) & PTA_K3_I2C_STATUS_FIFO_MASK)
		  << PTA_K3_I2C_STATUS_FIFO_SHIFT;

	params[1].value.a = packed;
	params[1].value.b = omap_i2c_get_speed_khz(dev);
	params[2].value.a = omap_i2c_get_slave_addr(dev);
	params[2].value.b = omap_i2c_get_inter_msg_delay_us(dev);
	params[3].value.a = ss->write_count;
	params[3].value.b = ss->read_count;

	return TEE_SUCCESS;
}

static TEE_Result cmd_init(uint32_t param_types,
			   TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);
	uint32_t instance;
	uint32_t speed_khz;
	uint32_t inter_msg_delay_us;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	speed_khz = params[0].value.b;
	inter_msg_delay_us = params[1].value.a;

	DMSG("CMD_INIT: instance=%" PRIu32 " speed=%" PRIu32
	     "kHz delay=%" PRIu32 "us",
	     instance, speed_khz, inter_msg_delay_us);

	return k3_i2c_init_instance(instance, speed_khz, inter_msg_delay_us);
}

static TEE_Result cmd_deinit(uint32_t param_types,
			     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);
	uint32_t instance;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;

	DMSG("CMD_DEINIT: instance=%" PRIu32, instance);

	return k3_i2c_deinit_instance(instance);
}

static TEE_Result cmd_simple_read(uint32_t param_types,
				  TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_NONE);
	struct omap_i2c_dev *dev;
	struct omap_i2c_msg msgs[2];
	uint8_t reg_addr;
	uint8_t buf[4] = { 0 };
	uint32_t instance, addr, reg, len;
	uint32_t packed = 0;
	TEE_Result res;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	addr = params[0].value.b;
	reg = params[1].value.a;
	len = params[1].value.b;

	if (len == 0 || len > 4)
		return TEE_ERROR_BAD_PARAMETERS;

	res = get_dev(instance, &dev);
	if (res)
		return res;

	reg_addr = (uint8_t)reg;

	/* Message 1: write register address */
	msgs[0].addr = (uint16_t)addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;

	/* Message 2: read data */
	msgs[1].addr = (uint16_t)addr;
	msgs[1].flags = OMAP_I2C_M_RD | OMAP_I2C_M_STOP;
	msgs[1].len = (uint16_t)len;
	msgs[1].buf = buf;

	res = omap_i2c_master_xfer(dev, msgs, 2);
	if (res)
		return res;

	for (uint32_t i = 0; i < len; i++)
		packed |= (uint32_t)buf[i] << (i * 8);

	params[2].value.a = packed;
	params[2].value.b = len;

	return TEE_SUCCESS;
}

static TEE_Result cmd_simple_write(uint32_t param_types,
				   TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_NONE);
	struct omap_i2c_dev *dev;
	struct omap_i2c_msg msg;
	uint8_t buf[4];
	uint32_t instance, addr, reg, data_packed, len;
	TEE_Result res;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	addr = params[0].value.b;
	reg = params[1].value.a;
	data_packed = params[2].value.a;
	len = params[2].value.b;

	if (len == 0 || len > 3)
		return TEE_ERROR_BAD_PARAMETERS;

	res = get_dev(instance, &dev);
	if (res)
		return res;

	buf[0] = (uint8_t)reg;
	for (uint32_t i = 0; i < len; i++)
		buf[1 + i] = (uint8_t)(data_packed >> (i * 8));

	msg.addr = (uint16_t)addr;
	msg.flags = OMAP_I2C_M_STOP;
	msg.len = (uint16_t)(1 + len);
	msg.buf = buf;

	return omap_i2c_master_xfer(dev, &msg, 1);
}

static TEE_Result cmd_slave_register(uint32_t param_types,
				     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);
	struct omap_i2c_dev *dev;
	struct slave_state *ss;
	uint32_t instance, addr;
	TEE_Result res;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	addr = params[0].value.b;

	res = get_dev(instance, &dev);
	if (res)
		return res;

	ss = &slave_states[instance];

	memset(ss->regs, 0, sizeof(ss->regs));
	ss->reg_ptr = 0;
	ss->first_byte = true;
	ss->is_read = false;
	ss->write_count = 0;
	ss->read_count = 0;
	ss->lock = SPINLOCK_UNLOCK;

	res = omap_i2c_slave_register(dev, (uint16_t)addr, false,
				      slave_callback, ss);
	if (res)
		return res;

	ss->active = true;

	DMSG("Slave registered: instance=%" PRIu32 " addr=0x%02" PRIx32,
	     instance, addr);

	return TEE_SUCCESS;
}

static TEE_Result cmd_slave_unregister(uint32_t param_types,
				       TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);
	struct omap_i2c_dev *dev;
	struct slave_state *ss;
	uint32_t instance;
	TEE_Result res;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;

	res = get_dev(instance, &dev);
	if (res)
		return res;

	ss = &slave_states[instance];
	ss->active = false;

	res = omap_i2c_slave_unregister(dev);
	if (res)
		return res;

	DMSG("Slave unregistered: instance=%" PRIu32, instance);

	return TEE_SUCCESS;
}

static TEE_Result cmd_slave_set_regs(uint32_t param_types,
				     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT);
	struct slave_state *ss;
	uint32_t instance, start, len;
	uint32_t lo, hi;
	uint32_t exceptions;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	start = params[1].value.a;
	len = params[1].value.b;
	lo = params[2].value.a;
	hi = params[3].value.a;

	if (len == 0 || len > 8 || start + len > SLAVE_REG_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	ss = &slave_states[instance];
	exceptions = cpu_spin_lock_xsave(&ss->lock);

	for (uint32_t i = 0; i < len && i < 4; i++)
		ss->regs[start + i] = (uint8_t)(lo >> (i * 8));
	for (uint32_t i = 4; i < len; i++)
		ss->regs[start + i] = (uint8_t)(hi >> ((i - 4) * 8));

	cpu_spin_unlock_xrestore(&ss->lock, exceptions);

	return TEE_SUCCESS;
}

static TEE_Result cmd_slave_get_regs(uint32_t param_types,
				     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT);
	struct slave_state *ss;
	uint32_t instance, start, len;
	uint32_t lo = 0, hi = 0;
	uint32_t exceptions;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	start = params[1].value.a;
	len = params[1].value.b;

	if (len == 0 || len > 8 || start + len > SLAVE_REG_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	ss = &slave_states[instance];
	exceptions = cpu_spin_lock_xsave(&ss->lock);

	for (uint32_t i = 0; i < len && i < 4; i++)
		lo |= (uint32_t)ss->regs[start + i] << (i * 8);
	for (uint32_t i = 4; i < len; i++)
		hi |= (uint32_t)ss->regs[start + i] << ((i - 4) * 8);

	cpu_spin_unlock_xrestore(&ss->lock, exceptions);

	params[2].value.a = lo;
	params[3].value.a = hi;

	return TEE_SUCCESS;
}

static TEE_Result cmd_slave_poll(uint32_t param_types,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t exp = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				       TEE_PARAM_TYPE_VALUE_OUTPUT,
				       TEE_PARAM_TYPE_NONE,
				       TEE_PARAM_TYPE_NONE);
	struct omap_i2c_dev *dev;
	struct slave_state *ss;
	uint32_t instance, timeout_ms;
	TEE_Result res;

	if (param_types != exp)
		return TEE_ERROR_BAD_PARAMETERS;

	instance = params[0].value.a;
	timeout_ms = params[0].value.b;

	res = get_dev(instance, &dev);
	if (res)
		return res;

	if (instance >= K3_I2C_MAX_INSTANCES)
		return TEE_ERROR_BAD_PARAMETERS;

	ss = &slave_states[instance];
	if (!ss->active)
		return TEE_ERROR_BAD_STATE;

	omap_i2c_slave_poll(dev, timeout_ms);

	params[1].value.a = ss->write_count;
	params[1].value.b = ss->read_count;

	return TEE_SUCCESS;
}

static TEE_Result pta_k3_i2c_invoke(uint32_t param_types,
				    TEE_Param params[TEE_NUM_PARAMS],
				    uint32_t cmd_id)
{
	switch (cmd_id) {
	case PTA_K3_I2C_CMD_PING:
		return cmd_ping(param_types, params);
	case PTA_K3_I2C_CMD_STATUS:
		return cmd_status(param_types, params);
	case PTA_K3_I2C_CMD_INIT:
		return cmd_init(param_types, params);
	case PTA_K3_I2C_CMD_DEINIT:
		return cmd_deinit(param_types, params);
	case PTA_K3_I2C_CMD_SIMPLE_READ:
		return cmd_simple_read(param_types, params);
	case PTA_K3_I2C_CMD_SIMPLE_WRITE:
		return cmd_simple_write(param_types, params);
	case PTA_K3_I2C_CMD_SLAVE_REGISTER:
		return cmd_slave_register(param_types, params);
	case PTA_K3_I2C_CMD_SLAVE_UNREGISTER:
		return cmd_slave_unregister(param_types, params);
	case PTA_K3_I2C_CMD_SLAVE_SET_REGS:
		return cmd_slave_set_regs(param_types, params);
	case PTA_K3_I2C_CMD_SLAVE_GET_REGS:
		return cmd_slave_get_regs(param_types, params);
	case PTA_K3_I2C_CMD_SLAVE_POLL:
		return cmd_slave_poll(param_types, params);
	default:
		return TEE_ERROR_NOT_IMPLEMENTED;
	}
}

static TEE_Result pta_k3_i2c_invoke_command(void *sess_ctx __unused,
					    uint32_t cmd_id,
					    uint32_t param_types,
					    TEE_Param params[TEE_NUM_PARAMS])
{
	return pta_k3_i2c_invoke(param_types, params, cmd_id);
}

pseudo_ta_register(.uuid = PTA_K3_I2C_UUID,
		   .name = PTA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = pta_k3_i2c_invoke_command);
