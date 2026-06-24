// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, BayLibre
 *
 * TI OMAP/K3 I2C controller driver
 */

#include <assert.h>
#include <drivers/omap_i2c.h>
#include <io.h>
#include <kernel/delay.h>
#include <kernel/spinlock.h>
#include <malloc.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <string.h>
#include <trace.h>
#include <util.h>

/* Register index enumeration */
enum omap_i2c_reg_idx {
	REG_REV = 0,
	REG_IE,
	REG_STAT,
	REG_IV,
	REG_WE,
	REG_SYSS,
	REG_BUF,
	REG_CNT,
	REG_DATA,
	REG_SYSC,
	REG_CON,
	REG_OA,
	REG_SA,
	REG_PSC,
	REG_SCLL,
	REG_SCLH,
	REG_SYSTEST,
	REG_BUFSTAT,
	REG_IP_V2_REVNB_LO,
	REG_IP_V2_REVNB_HI,
	REG_IP_V2_IRQSTATUS_RAW,
	REG_IP_V2_IRQENABLE_SET,
	REG_IP_V2_IRQENABLE_CLR,
	REG__COUNT,
};

/* Register offset maps */
static const uint16_t reg_map_v1[REG__COUNT] = {
	[REG_REV]     = 0x00,
	[REG_IE]      = 0x04,
	[REG_STAT]    = 0x08,
	[REG_IV]      = 0x0c,
	[REG_WE]      = 0x0c,
	[REG_SYSS]    = 0x10,
	[REG_BUF]     = 0x14,
	[REG_CNT]     = 0x18,
	[REG_DATA]    = 0x1c,
	[REG_SYSC]    = 0x20,
	[REG_CON]     = 0x24,
	[REG_OA]      = 0x28,
	[REG_SA]      = 0x2c,
	[REG_PSC]     = 0x30,
	[REG_SCLL]    = 0x34,
	[REG_SCLH]    = 0x38,
	[REG_SYSTEST] = 0x3c,
	[REG_BUFSTAT] = 0x40,
};

static const uint16_t reg_map_v2[REG__COUNT] = {
	[REG_REV]		  = 0x04,
	[REG_IE]		  = 0x2c,
	[REG_STAT]		  = 0x28,
	[REG_IV]		  = 0x34,
	[REG_WE]		  = 0x34,
	[REG_SYSS]		  = 0x90,
	[REG_BUF]		  = 0x94,
	[REG_CNT]		  = 0x98,
	[REG_DATA]		  = 0x9c,
	[REG_SYSC]		  = 0x10,
	[REG_CON]		  = 0xa4,
	[REG_OA]		  = 0xa8,
	[REG_SA]		  = 0xac,
	[REG_PSC]		  = 0xb0,
	[REG_SCLL]		  = 0xb4,
	[REG_SCLH]		  = 0xb8,
	[REG_SYSTEST]		  = 0xbc,
	[REG_BUFSTAT]		  = 0xc0,
	[REG_IP_V2_REVNB_LO]	  = 0x00,
	[REG_IP_V2_REVNB_HI]	  = 0x04,
	[REG_IP_V2_IRQSTATUS_RAW] = 0x24,
	[REG_IP_V2_IRQENABLE_SET] = 0x2c,
	[REG_IP_V2_IRQENABLE_CLR] = 0x30,
};

/* Platform flags */
#define OMAP_I2C_FLAG_NO_FIFO			BIT(0)
#define OMAP_I2C_FLAG_SIMPLE_CLOCK		BIT(1)
#define OMAP_I2C_FLAG_16BIT_DATA_REG		BIT(2)
#define OMAP_I2C_FLAG_FORCE_19200_INT_CLK	BIT(3)
#define OMAP_I2C_FLAG_BUS_SHIFT__SHIFT		13
#define OMAP_I2C_FLAG_BUS_SHIFT_2		(2 << OMAP_I2C_FLAG_BUS_SHIFT__SHIFT)

/* STAT / IE */
#define STAT_XDR	BIT(14)
#define STAT_RDR	BIT(13)
#define STAT_BB		BIT(12)
#define STAT_ROVR	BIT(11)
#define STAT_XUDF	BIT(10)
#define STAT_AAS	BIT(9)
#define STAT_BF		BIT(8)
#define STAT_XRDY	BIT(4)
#define STAT_RRDY	BIT(3)
#define STAT_ARDY	BIT(2)
#define STAT_NACK	BIT(1)
#define STAT_AL		BIT(0)

/* BUF */
#define BUF_RXFIF_CLR	BIT(14)
#define BUF_TXFIF_CLR	BIT(6)

/* CON */
#define CON_EN		BIT(15)
#define CON_OPMODE_HS	BIT(12)
#define CON_MST		BIT(10)
#define CON_TRX		BIT(9)
#define CON_XA		BIT(8)
#define CON_XOA		BIT(7)
#define CON_STP		BIT(1)
#define CON_STT		BIT(0)

/* SYSS */
#define SYSS_RESETDONE	BIT(0)

/* SYSC */
#define SYSC_SRST	BIT(1)

/* BUFSTAT shifts (IP v2) */
#define BUFSTAT_TXSTAT_SHIFT	0
#define BUFSTAT_TXSTAT_MASK	0x3f
#define BUFSTAT_RXSTAT_SHIFT	8
#define BUFSTAT_RXSTAT_MASK	0x3f
#define BUFSTAT_FIFODEPTH_SHIFT	14
#define BUFSTAT_FIFODEPTH_MASK	0x3

/* WE all bits */
#define WE_ALL			0x636f

/* Timeouts */
#define TIMEOUT_US		(1000 * 1000)
#define BUS_FREE_TIMEOUT_US	(100 * 1000)
#define RESET_TIMEOUT_US	(10 * 1000)
#define POLL_DELAY_US		5

/* FIFO */
#define MAX_FIFO_SIZE		64

/* Default functional clock rate */
#define DEFAULT_FCLK_RATE	48000000

/* Errata */
#define ERRATA_I207		BIT(0)
#define ERRATA_I462		BIT(1)

/* Scheme */
#define SCHEME_0		0
#define SCHEME_1		1

struct omap_i2c_dev {
	vaddr_t base;
	paddr_t base_pa;
	size_t base_sz;

	const uint16_t *regs;
	uint32_t ip_rev;
	uint32_t flags;
	uint32_t errata;

	uint32_t speed;		/* bus speed in kHz */
	uint32_t fclk_rate;	/* functional clock in Hz */

	uint16_t fifo_size;	/* bytes per FIFO */
	uint16_t threshold;

	uint16_t pscstate;
	uint16_t scllstate;
	uint16_t sclhstate;

	uint16_t iestate;
	uint16_t westate;
	uint32_t inter_msg_delay_us;

	/* Transfer state */
	struct omap_i2c_msg *msg;
	size_t msg_idx;
	size_t msg_num;
	uint8_t *buf;
	size_t buf_len;
	bool receiver;

	/* Slave state */
	bool slave_active;
	uint16_t slave_addr;
	bool slave_10bit;
	omap_i2c_slave_cb_t slave_cb;
	void *slave_cb_data;
	bool slave_idle;
	bool slave_read_started;

	unsigned int lock;
};

static inline uint16_t omap_i2c_read(struct omap_i2c_dev *dev,
				     enum omap_i2c_reg_idx idx)
{
	return io_read16(dev->base + dev->regs[idx]);
}

static inline void omap_i2c_write(struct omap_i2c_dev *dev,
				  enum omap_i2c_reg_idx idx, uint16_t val)
{
	io_write16(dev->base + dev->regs[idx], val);
}

static void omap_i2c_flush_fifo(struct omap_i2c_dev *dev)
{
	uint16_t stat;
	uint64_t deadline = timeout_init_us(1000);

	while (!timeout_elapsed(deadline)) {
		stat = omap_i2c_read(dev, REG_STAT);
		if (!(stat & STAT_RRDY))
			break;
		(void)omap_i2c_read(dev, REG_DATA);
		omap_i2c_write(dev, REG_STAT, STAT_RRDY);
	}
}

static TEE_Result omap_i2c_reset(struct omap_i2c_dev *dev)
{
	uint64_t deadline;

	/* Disable controller */
	omap_i2c_write(dev, REG_CON, 0);

	if (dev->ip_rev == OMAP_I2C_IP_VERSION_2) {
		omap_i2c_write(dev, REG_SYSC, SYSC_SRST);
		/* enable to kick reset */
		omap_i2c_write(dev, REG_CON, CON_EN);

		deadline = timeout_init_us(RESET_TIMEOUT_US);
		while (!(omap_i2c_read(dev, REG_SYSS) & SYSS_RESETDONE)) {
			if (timeout_elapsed(deadline)) {
				EMSG("I2C reset timeout");
				return TEE_ERROR_BUSY;
			}
			udelay(POLL_DELAY_US);
		}

		/* Disable again for configuration */
		omap_i2c_write(dev, REG_CON, 0);
	} else {
		omap_i2c_write(dev, REG_SYSC, SYSC_SRST);

		omap_i2c_write(dev, REG_CON, CON_EN);

		deadline = timeout_init_us(RESET_TIMEOUT_US);
		while (!(omap_i2c_read(dev, REG_SYSS) & SYSS_RESETDONE)) {
			if (timeout_elapsed(deadline)) {
				EMSG("I2C reset timeout");
				return TEE_ERROR_BUSY;
			}
			udelay(POLL_DELAY_US);
		}

		omap_i2c_write(dev, REG_CON, 0);
	}

	return TEE_SUCCESS;
}

static void omap_i2c_compute_clk(struct omap_i2c_dev *dev)
{
	uint32_t fclk = dev->fclk_rate;
	uint32_t speed = dev->speed * 1000; /* kHz -> Hz */
	uint32_t internal_clk;
	uint32_t psc;
	uint32_t scl;
	uint32_t scll;
	uint32_t sclh;

	if (speed <= 100000)
		internal_clk = 4000000;
	else if (speed <= 400000)
		internal_clk = 9600000;
	else
		internal_clk = 19200000;

	psc = (fclk / internal_clk) - 1;
	if (psc > 0xff)
		psc = 0xff;

	/* SCL total ticks = internal_clk / speed */
	scl = internal_clk / speed;
	if (scl < 7)
		scl = 7;

	/* Split roughly 50/50, low slightly longer */
	scll = (scl - 7) / 2;
	sclh = scl - 7 - scll;

	dev->pscstate = (uint16_t)psc;
	dev->scllstate = (uint16_t)scll;
	dev->sclhstate = (uint16_t)sclh;
}

static void omap_i2c_setup_fifo(struct omap_i2c_dev *dev)
{
	uint16_t buf_stat;
	uint16_t fifo_depth;

	if (dev->flags & OMAP_I2C_FLAG_NO_FIFO) {
		dev->fifo_size = 1;
		dev->threshold = 1;
		return;
	}

	if (dev->ip_rev == OMAP_I2C_IP_VERSION_2) {
		/* IP v2: FIFO depth encoded in BUFSTAT[15:14] */
		omap_i2c_write(dev, REG_CON, CON_EN);
		buf_stat = omap_i2c_read(dev, REG_BUFSTAT);
		fifo_depth = (buf_stat >> BUFSTAT_FIFODEPTH_SHIFT) &
			     BUFSTAT_FIFODEPTH_MASK;
		/* fifo_depth encoding: 0=8, 1=16, 2=32, 3=64 */
		dev->fifo_size = 8 << fifo_depth;
		omap_i2c_write(dev, REG_CON, 0);
	} else {
		dev->fifo_size = 1;
	}

	if (dev->fifo_size > MAX_FIFO_SIZE)
		dev->fifo_size = MAX_FIFO_SIZE;

	dev->threshold = dev->fifo_size / 2;
	if (dev->threshold < 1)
		dev->threshold = 1;
}

static void omap_i2c_hw_init(struct omap_i2c_dev *dev)
{
	omap_i2c_write(dev, REG_CON, 0);

	omap_i2c_write(dev, REG_PSC, dev->pscstate);
	omap_i2c_write(dev, REG_SCLL, dev->scllstate);
	omap_i2c_write(dev, REG_SCLH, dev->sclhstate);

	if (dev->westate)
		omap_i2c_write(dev, REG_WE, dev->westate);

	omap_i2c_write(dev, REG_CON, CON_EN);

	if (dev->iestate)
		omap_i2c_write(dev, REG_IE, dev->iestate);
}

static inline void omap_i2c_ack_stat(struct omap_i2c_dev *dev, uint16_t mask)
{
	omap_i2c_write(dev, REG_STAT, mask);
}

static TEE_Result omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
	uint64_t deadline = timeout_init_us(TIMEOUT_US);

	while (omap_i2c_read(dev, REG_STAT) & STAT_BB) {
		if (timeout_elapsed(deadline)) {
			EMSG("I2C bus busy timeout");
			return TEE_ERROR_BUSY;
		}
		udelay(POLL_DELAY_US);
	}

	return TEE_SUCCESS;
}

static void omap_i2c_resize_fifo(struct omap_i2c_dev *dev, uint16_t size,
				 bool is_rx)
{
	uint16_t buf;

	if (dev->flags & OMAP_I2C_FLAG_NO_FIFO)
		return;

	dev->threshold = size;
	if (dev->threshold > dev->fifo_size)
		dev->threshold = dev->fifo_size;
	if (dev->threshold < 1)
		dev->threshold = 1;

	buf = omap_i2c_read(dev, REG_BUF);

	if (is_rx) {
		buf &= ~(0x3f << 8);
		buf |= ((dev->threshold - 1) << 8) | BUF_RXFIF_CLR;
	} else {
		buf &= ~0x3f;
		buf |= (dev->threshold - 1) | BUF_TXFIF_CLR;
	}

	omap_i2c_write(dev, REG_BUF, buf);
}

/* ------------------------------------------------------------------ */
/* Errata i462: wait for XUDF before writing DATA		      */
/* ------------------------------------------------------------------ */
static TEE_Result errata_omap3_i462(struct omap_i2c_dev *dev)
{
	unsigned int timeout = 10000;
	uint16_t stat;

	do {
		stat = omap_i2c_read(dev, REG_STAT);
		if (stat & STAT_XUDF)
			break;

		if (stat & (STAT_NACK | STAT_AL)) {
			omap_i2c_ack_stat(dev, STAT_XRDY | STAT_XDR);
			if (stat & STAT_NACK)
				omap_i2c_ack_stat(dev, STAT_NACK);
			if (stat & STAT_AL)
				omap_i2c_ack_stat(dev, STAT_AL);
			return TEE_ERROR_COMMUNICATION;
		}
	} while (--timeout);

	if (!timeout) {
		EMSG("Timeout waiting on XUDF");
		return TEE_ERROR_COMMUNICATION;
	}

	return TEE_SUCCESS;
}

static void omap_i2c_receive_data(struct omap_i2c_dev *dev, uint8_t num_bytes)
{
	uint16_t w;

	while (num_bytes-- && dev->buf_len) {
		w = omap_i2c_read(dev, REG_DATA);
		*dev->buf++ = (uint8_t)w;
		dev->buf_len--;

		if ((dev->flags & OMAP_I2C_FLAG_16BIT_DATA_REG) &&
		    dev->buf_len) {
			*dev->buf++ = (uint8_t)(w >> 8);
			dev->buf_len--;
		}
	}
}

static TEE_Result omap_i2c_transmit_data(struct omap_i2c_dev *dev,
					 uint8_t num_bytes)
{
	uint16_t w;
	TEE_Result ret;

	while (num_bytes-- && dev->buf_len) {
		w = *dev->buf++;
		dev->buf_len--;

		if ((dev->flags & OMAP_I2C_FLAG_16BIT_DATA_REG) &&
		    dev->buf_len) {
			w |= (uint16_t)(*dev->buf++) << 8;
			dev->buf_len--;
		}

		if (dev->errata & ERRATA_I462) {
			ret = errata_omap3_i462(dev);
			if (ret)
				return ret;
		}

		omap_i2c_write(dev, REG_DATA, w);
	}

	return TEE_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Poll-drive a single data phase (called in a loop)		      */
/* Returns: 0 = done, 1 = keep polling, <0 = error bits		      */
/* ------------------------------------------------------------------ */
static int omap_i2c_poll_xfer_data(struct omap_i2c_dev *dev)
{
	uint16_t bits;
	uint16_t stat;
	int err = 0;
	int count = 0;

	do {
		bits = omap_i2c_read(dev, REG_IE);
		stat = omap_i2c_read(dev, REG_STAT);
		stat &= bits;

		if (dev->receiver)
			stat &= ~(STAT_XDR | STAT_XRDY);
		else
			stat &= ~(STAT_RDR | STAT_RRDY);

		if (!stat)
			return 1;

		if (count++ == 100)
			break;

		if (stat & STAT_NACK) {
			err |= STAT_NACK;
			omap_i2c_ack_stat(dev, STAT_NACK);
		}

		if (stat & STAT_AL) {
			err |= STAT_AL;
			omap_i2c_ack_stat(dev, STAT_AL);
		}

		if (stat & STAT_ROVR) {
			err |= STAT_ROVR;
			omap_i2c_ack_stat(dev, STAT_ROVR);
			return -err;
		}

		if (stat & STAT_XUDF) {
			err |= STAT_XUDF;
			omap_i2c_ack_stat(dev, STAT_XUDF);
			return -err;
		}

		if (err) {
			omap_i2c_ack_stat(dev, STAT_RRDY | STAT_RDR |
					  STAT_XRDY | STAT_XDR |
					  STAT_ARDY | STAT_BF);
			return -err;
		}

		/* Handle RX data */
		if (stat & STAT_RDR) {
			uint8_t num = dev->buf_len;

			if (num > dev->fifo_size)
				num = dev->fifo_size;
			if (!num)
				num = 1;
			omap_i2c_receive_data(dev, num);
			omap_i2c_ack_stat(dev, STAT_RDR);
			continue;
		}

		if (stat & STAT_RRDY) {
			uint8_t num = dev->threshold;

			if (!num)
				num = 1;
			omap_i2c_receive_data(dev, num);
			omap_i2c_ack_stat(dev, STAT_RRDY);
			continue;
		}

		/* Handle TX data using BUFSTAT to know FIFO space */
		if (stat & (STAT_XDR | STAT_XRDY)) {
			if (dev->buf_len) {
				uint16_t bufstat;
				uint8_t tx_fifo_level;
				uint8_t space;
				uint8_t num;

				bufstat = omap_i2c_read(dev, REG_BUFSTAT);
				tx_fifo_level = bufstat & BUFSTAT_TXSTAT_MASK;
				space = dev->fifo_size - tx_fifo_level;

				num = dev->buf_len;
				if (num > space)
					num = space;
				if (num == 0)
					num = 1;

				omap_i2c_transmit_data(dev, num);
			}
			omap_i2c_ack_stat(dev,
					  stat & (STAT_XDR | STAT_XRDY));
			continue;
		}

		if (stat & STAT_ARDY) {
			omap_i2c_ack_stat(dev, STAT_ARDY);
			return 0;
		}

		if (stat & STAT_BF) {
			omap_i2c_ack_stat(dev, STAT_BF);
			return 0;
		}
	} while (stat);

	return err ? -err : 1;
}

static TEE_Result omap_i2c_xfer_msg(struct omap_i2c_dev *dev,
				    struct omap_i2c_msg *msg, bool stop)
{
	uint16_t w;
	uint64_t deadline;
	int ret;

	DMSG("addr=0x%04x len=%u flags=0x%04x stop=%d",
	     msg->addr, msg->len, msg->flags, stop);

	dev->receiver = !!(msg->flags & OMAP_I2C_M_RD);
	omap_i2c_resize_fifo(dev, msg->len, dev->receiver);

	omap_i2c_write(dev, REG_SA, msg->addr);

	dev->buf = msg->buf;
	dev->buf_len = msg->len;

	omap_i2c_write(dev, REG_CNT, (uint16_t)dev->buf_len);

	/* Clear FIFOs */
	w = omap_i2c_read(dev, REG_BUF);
	w |= BUF_RXFIF_CLR | BUF_TXFIF_CLR;
	omap_i2c_write(dev, REG_BUF, w);

	/* Clear any pending status */
	omap_i2c_write(dev, REG_STAT, 0xffff);

	w = CON_EN | CON_MST | CON_STT;

	if (dev->speed > 400)
		w |= CON_OPMODE_HS;
	if (msg->flags & OMAP_I2C_M_STOP)
		stop = true;
	if (msg->flags & OMAP_I2C_M_TEN)
		w |= CON_XA;
	if (!(msg->flags & OMAP_I2C_M_RD))
		w |= CON_TRX;
	if (stop)
		w |= CON_STP;

	omap_i2c_write(dev, REG_CON, w);

	if (!dev->receiver) {
		/*
		 * TX: wait for address phase to complete, then
		 * load data into FIFO.
		 * At 400kHz: START + 8-bit addr + ACK = ~25us
		 */
		udelay(50);

		/* Now load all data into FIFO */
		if (dev->buf_len) {
			uint8_t num = dev->buf_len;

			if (num > dev->fifo_size)
				num = dev->fifo_size;
			omap_i2c_transmit_data(dev, num);
		}
	}

	deadline = timeout_init_us(TIMEOUT_US);
	do {
		ret = omap_i2c_poll_xfer_data(dev);
		if (ret <= 0)
			break;
		if (timeout_elapsed(deadline)) {
			EMSG("I2C transfer timeout");
			omap_i2c_reset(dev);
			omap_i2c_hw_init(dev);
			return TEE_ERROR_COMMUNICATION;
		}
		udelay(5);
	} while (true);

	if (ret >= 0 && stop) {
		uint64_t bb_deadline = timeout_init_us(50000);

		while (omap_i2c_read(dev, REG_STAT) & STAT_BB) {
			if (timeout_elapsed(bb_deadline))
				break;
			udelay(5);
		}
		udelay(100);
	}

	if (ret < 0) {
		uint16_t err_bits = (uint16_t)(-ret);

		if (err_bits & (STAT_ROVR | STAT_XUDF)) {
			omap_i2c_reset(dev);
			omap_i2c_hw_init(dev);
			return TEE_ERROR_COMMUNICATION;
		}

		if (err_bits & STAT_AL)
			return TEE_ERROR_BUSY;

		if (err_bits & STAT_NACK) {
			if (msg->flags & OMAP_I2C_M_IGNORE_NAK)
				return TEE_SUCCESS;

			w = omap_i2c_read(dev, REG_CON);
			w |= CON_STP;
			omap_i2c_write(dev, REG_CON, w);
			return TEE_ERROR_COMMUNICATION;
		}

		return TEE_ERROR_COMMUNICATION;
	}

	return TEE_SUCCESS;
}

TEE_Result omap_i2c_master_xfer(struct omap_i2c_dev *dev,
				struct omap_i2c_msg *msgs, size_t num)
{
	TEE_Result res;
	size_t i;
	uint32_t exceptions;

	if (!dev || !msgs || !num)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = cpu_spin_lock_xsave(&dev->lock);

	res = omap_i2c_wait_for_bb(dev);
	if (res)
		goto out;

	for (i = 0; i < num; i++) {
		/*
		 * Optional inter-message delay for repeated start sequences.
		 *
		 * In a multi-message I2C transfer (e.g. write register address
		 * then read data), the master holds the bus between messages
		 * and issues a repeated START. Hardware slaves handle this
		 * instantly, but software-emulated slaves (such as the Linux
		 * kernel slave-24c02 driver) need extra time to process the
		 * first message (register address) before they can respond
		 * to the read. Without this delay, the slave may not have
		 * updated its internal address pointer, causing a one-byte
		 * shift in the returned data.
		 */
		if (i > 0 && dev->inter_msg_delay_us)
			udelay(dev->inter_msg_delay_us);

		res = omap_i2c_xfer_msg(dev, &msgs[i], (i == (num - 1)));
		if (res)
			break;
	}

	/* Wait for bus free after last message */
	omap_i2c_wait_for_bb(dev);

out:
	cpu_spin_unlock_xrestore(&dev->lock, exceptions);
	return res;
}

static void omap_i2c_slave_rx_drain(struct omap_i2c_dev *dev)
{
	uint16_t bufstat;
	uint8_t rx_count;
	uint8_t value;

	bufstat = omap_i2c_read(dev, REG_BUFSTAT);
	rx_count = (bufstat >> BUFSTAT_RXSTAT_SHIFT) & BUFSTAT_RXSTAT_MASK;
	if (rx_count == 0)
		rx_count = 1;

	while (rx_count--) {
		value = (uint8_t)omap_i2c_read(dev, REG_DATA);
		if (dev->slave_cb)
			dev->slave_cb(OMAP_I2C_SLAVE_WRITE_RECEIVED,
				      &value, dev->slave_cb_data);
	}
}

static void omap_i2c_slave_event(struct omap_i2c_dev *dev)
{
	uint16_t bits;
	uint16_t stat;
	uint8_t value = 0;

	do {
		bits = omap_i2c_read(dev, REG_IE);
		stat = omap_i2c_read(dev, REG_STAT);
		stat &= bits;

		if (!stat)
			break;

		DMSG("Slave IRQ stat=0x%04x", stat);

		if (stat & STAT_AAS) {
			dev->slave_idle = true;
			dev->slave_read_started = false;
			omap_i2c_ack_stat(dev, STAT_AAS);
		}

		if (stat & STAT_RRDY) {
			if (dev->slave_idle) {
				dev->slave_idle = false;
				if (dev->slave_cb)
					dev->slave_cb(
						OMAP_I2C_SLAVE_WRITE_REQUESTED,
						&value, dev->slave_cb_data);
			}

			omap_i2c_slave_rx_drain(dev);
			omap_i2c_ack_stat(dev, STAT_RRDY);
			continue;
		}

		if (stat & STAT_XRDY) {
			if (!dev->slave_read_started) {
				dev->slave_read_started = true;
				if (dev->slave_cb)
					dev->slave_cb(
						OMAP_I2C_SLAVE_READ_REQUESTED,
						&value, dev->slave_cb_data);
			} else {
				if (dev->slave_cb)
					dev->slave_cb(
						OMAP_I2C_SLAVE_READ_PROCESSED,
						&value, dev->slave_cb_data);
			}

			omap_i2c_ack_stat(dev, STAT_XRDY);
			omap_i2c_write(dev, REG_DATA, (uint16_t)value);
			continue;
		}

		if (stat & (STAT_ARDY | STAT_NACK)) {
			omap_i2c_ack_stat(dev,
					  STAT_ARDY | STAT_NACK);
		}

		if (stat & STAT_BF) {
			dev->slave_idle = true;
			dev->slave_read_started = false;
			if (dev->slave_cb)
				dev->slave_cb(OMAP_I2C_SLAVE_STOP,
					      &value, dev->slave_cb_data);

			omap_i2c_ack_stat(dev, STAT_BF);
		}
	} while (stat);
}

TEE_Result omap_i2c_slave_register(struct omap_i2c_dev *dev, uint16_t addr,
				   bool is_10bit, omap_i2c_slave_cb_t cb,
				   void *cb_data)
{
	uint16_t con;
	uint32_t exceptions;

	if (!dev || !cb)
		return TEE_ERROR_BAD_PARAMETERS;

	if (dev->slave_active)
		return TEE_ERROR_BUSY;

	exceptions = cpu_spin_lock_xsave(&dev->lock);

	dev->slave_addr = addr;
	dev->slave_10bit = is_10bit;
	dev->slave_cb = cb;
	dev->slave_cb_data = cb_data;

	/* Disable controller for reconfiguration */
	omap_i2c_write(dev, REG_CON, 0);

	/* Set own address */
	omap_i2c_write(dev, REG_OA, addr);

	/* Clear any pending status */
	omap_i2c_write(dev, REG_STAT, 0xffff);

	/* Clear FIFOs */
	omap_i2c_write(dev, REG_BUF, BUF_RXFIF_CLR | BUF_TXFIF_CLR);

	/* Keep clock configuration (PSC/SCLL/SCLH already set by init) */

	/* Enable controller in SLAVE mode (no CON_MST) */
	con = CON_EN;
	if (is_10bit)
		con |= CON_XOA;
	omap_i2c_write(dev, REG_CON, con);

	/* Enable slave-relevant interrupts */
	dev->iestate = STAT_AAS | STAT_BF | STAT_XRDY | STAT_RRDY |
		       STAT_ARDY | STAT_NACK;
	omap_i2c_write(dev, REG_IE, dev->iestate);

	dev->slave_active = true;
	dev->slave_idle = true;
	dev->slave_read_started = false;

	DMSG("Slave configured: CON=0x%04x OA=0x%04x IE=0x%04x",
	     omap_i2c_read(dev, REG_CON),
	     omap_i2c_read(dev, REG_OA),
	     omap_i2c_read(dev, REG_IE));

	cpu_spin_unlock_xrestore(&dev->lock, exceptions);

	return TEE_SUCCESS;
}

TEE_Result omap_i2c_slave_unregister(struct omap_i2c_dev *dev)
{
	uint32_t exceptions;

	if (!dev)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!dev->slave_active)
		return TEE_SUCCESS;

	exceptions = cpu_spin_lock_xsave(&dev->lock);

	/* Disable controller */
	omap_i2c_write(dev, REG_CON, 0);

	/* Clear own address */
	omap_i2c_write(dev, REG_OA, 0);

	/* Clear pending status */
	omap_i2c_write(dev, REG_STAT, 0xffff);

	/* Restore master-mode IE state */
	dev->iestate = STAT_XRDY | STAT_RRDY | STAT_ARDY |
		       STAT_NACK | STAT_AL;
	if (!(dev->flags & OMAP_I2C_FLAG_NO_FIFO))
		dev->iestate |= STAT_XDR | STAT_RDR;

	/* Re-enable in master mode */
	omap_i2c_write(dev, REG_CON, CON_EN);
	omap_i2c_write(dev, REG_IE, dev->iestate);

	dev->slave_active = false;
	dev->slave_cb = NULL;
	dev->slave_cb_data = NULL;

	DMSG("I2C slave unregistered, restored master mode");

	cpu_spin_unlock_xrestore(&dev->lock, exceptions);

	return TEE_SUCCESS;
}

TEE_Result omap_i2c_init(paddr_t base_pa, size_t base_sz,
			 const struct omap_i2c_platform_data *pdata,
			 struct omap_i2c_dev **out_dev)
{
	struct omap_i2c_dev *dev;
	TEE_Result res;
	uint16_t rev;
	uint16_t scheme;

	if (!pdata || !out_dev || !base_pa || !base_sz)
		return TEE_ERROR_BAD_PARAMETERS;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return TEE_ERROR_OUT_OF_MEMORY;

	dev->base_pa = base_pa;
	dev->base_sz = base_sz;
	dev->base = (vaddr_t)core_mmu_add_mapping(MEM_AREA_IO_SEC,
						  base_pa, base_sz);
	if (!dev->base) {
		EMSG("Failed to map I2C registers at PA 0x%lx",
		     (unsigned long)base_pa);
		free(dev);
		return TEE_ERROR_GENERIC;
	}

	dev->ip_rev = pdata->ip_rev;
	dev->flags = pdata->flags;
	dev->speed = pdata->clkrate ? pdata->clkrate : 100;
	dev->fclk_rate = pdata->fclk_rate ? pdata->fclk_rate :
			 DEFAULT_FCLK_RATE;
	dev->inter_msg_delay_us = pdata->inter_msg_delay_us;
	dev->lock = SPINLOCK_UNLOCK;

	/* Detect register map from IP revision */
	rev = io_read16(dev->base + 0x04);
	scheme = (rev >> 14) & 0x3;

	if (scheme == SCHEME_1 ||
	    dev->ip_rev == OMAP_I2C_IP_VERSION_2) {
		dev->regs = reg_map_v2;
		DMSG("I2C using IP v2 register map");
	} else {
		dev->regs = reg_map_v1;
		DMSG("I2C using IP v1 register map");
	}

	dev->errata = 0;

	if (dev->ip_rev == OMAP_I2C_IP_VERSION_2)
		dev->westate = WE_ALL;

	omap_i2c_compute_clk(dev);

	/* Master-mode IE state */
	dev->iestate = STAT_XRDY | STAT_RRDY | STAT_ARDY |
		       STAT_NACK | STAT_AL;
	if (!(dev->flags & OMAP_I2C_FLAG_NO_FIFO))
		dev->iestate |= STAT_XDR | STAT_RDR;

	/* Reset controller */
	res = omap_i2c_reset(dev);
	if (res) {
		free(dev);
		return res;
	}

	/* Detect FIFO */
	omap_i2c_setup_fifo(dev);

	/* Apply clock and enable */
	omap_i2c_hw_init(dev);

	/* Flush any stale data */
	omap_i2c_flush_fifo(dev);

	/* Clear pending status */
	omap_i2c_write(dev, REG_STAT, 0xffff);

	DMSG("OMAP I2C initialized at PA 0x%lx speed=%lukHz fifo=%u",
	     (unsigned long)base_pa, (unsigned long)dev->speed,
	     dev->fifo_size);

	*out_dev = dev;
	return TEE_SUCCESS;
}

void omap_i2c_cleanup(struct omap_i2c_dev *dev)
{
	if (!dev)
		return;

	if (dev->slave_active)
		omap_i2c_slave_unregister(dev);

	/* Disable controller */
	omap_i2c_write(dev, REG_IE, 0);
	omap_i2c_write(dev, REG_CON, 0);

	free(dev);
}

TEE_Result omap_i2c_slave_poll(struct omap_i2c_dev *dev, uint32_t timeout_ms)
{
	uint16_t stat;
	uint16_t mask;
	bool handled = false;
	uint64_t deadline;

	if (!dev || !dev->slave_active)
		return TEE_ERROR_BAD_STATE;

	deadline = timeout_init_us((uint64_t)timeout_ms * 1000);

	do {
		mask = omap_i2c_read(dev, REG_IE);
		stat = omap_i2c_read(dev, REG_STAT);
		stat &= mask;

		if (stat & (STAT_AAS | STAT_RRDY | STAT_XRDY | STAT_BF)) {
			omap_i2c_slave_event(dev);
			handled = true;
		}

		if (timeout_ms == 0)
			break;
	} while (!timeout_elapsed(deadline));

	return handled ? TEE_SUCCESS : TEE_ERROR_NO_DATA;
}

bool omap_i2c_is_slave_active(struct omap_i2c_dev *dev)
{
	return dev ? dev->slave_active : false;
}

uint16_t omap_i2c_get_slave_addr(struct omap_i2c_dev *dev)
{
	return dev ? dev->slave_addr : 0;
}

uint32_t omap_i2c_get_speed_khz(struct omap_i2c_dev *dev)
{
	return dev ? dev->speed : 0;
}

uint32_t omap_i2c_get_inter_msg_delay_us(struct omap_i2c_dev *dev)
{
	return dev ? dev->inter_msg_delay_us : 0;
}

uint16_t omap_i2c_get_fifo_size(struct omap_i2c_dev *dev)
{
	return dev ? dev->fifo_size : 0;
}

bool omap_i2c_is_bus_busy(struct omap_i2c_dev *dev)
{
	if (!dev)
		return false;

	return !!(omap_i2c_read(dev, REG_STAT) & STAT_BB);
}
