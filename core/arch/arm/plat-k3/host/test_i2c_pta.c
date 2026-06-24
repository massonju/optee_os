// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, BayLibre
 *
 * Test tool for K3 I2C PTA — master and slave modes
 * Uses only value params (no shared memory).
 *
 * $ aarch64-none-linux-gnu-gcc -o test_i2c_pta test_i2c_pta.c -static
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/tee.h>

#define TEE_DEVICE "/dev/tee0"

static uint8_t uuid[] = {
	0xa6, 0xb1, 0xe0, 0x7e,
	0x4a, 0x1e,
	0x4b, 0x5d,
	0x8c, 0x3a, 0x2b, 0x9f, 0x5e, 0x6d, 0x7c, 0x8a
};

/* Command IDs — must match pta_k3_i2c.h */
#define CMD_PING		0
#define CMD_STATUS		1
#define CMD_INIT		2
#define CMD_DEINIT		3
#define CMD_SIMPLE_READ		4
#define CMD_SIMPLE_WRITE	5
#define CMD_SLAVE_REGISTER	6
#define CMD_SLAVE_UNREGISTER	7
#define CMD_SLAVE_SET_REGS	8
#define CMD_SLAVE_GET_REGS	9
#define CMD_SLAVE_POLL		10

static int invoke(int fd, uint32_t session, uint32_t cmd,
		  struct tee_ioctl_param *p)
{
	struct {
		struct tee_ioctl_invoke_arg arg;
		struct tee_ioctl_param params[4];
	} data;
	struct tee_ioctl_buf_data buf_data;

	memset(&data, 0, sizeof(data));
	data.arg.func = cmd;
	data.arg.session = session;
	data.arg.num_params = 4;
	memcpy(data.params, p, sizeof(data.params));

	buf_data.buf_ptr = (uintptr_t)&data;
	buf_data.buf_len = sizeof(data);

	if (ioctl(fd, TEE_IOC_INVOKE, &buf_data) < 0) {
		perror("INVOKE");
		return -1;
	}

	if (data.arg.ret) {
		fprintf(stderr, "cmd %u failed: ret=0x%x origin=0x%x\n",
			cmd, data.arg.ret, data.arg.ret_origin);
		return -1;
	}

	memcpy(p, data.params, sizeof(data.params));
	return 0;
}

static int open_session(int fd, uint32_t *session)
{
	struct {
		struct tee_ioctl_open_session_arg arg;
		struct tee_ioctl_param params[0];
	} sess;
	struct tee_ioctl_buf_data buf_data;

	memset(&sess, 0, sizeof(sess));
	memcpy(sess.arg.uuid, uuid, 16);
	sess.arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess.arg.num_params = 0;

	buf_data.buf_ptr = (uintptr_t)&sess;
	buf_data.buf_len = sizeof(sess);

	if (ioctl(fd, TEE_IOC_OPEN_SESSION, &buf_data) < 0) {
		perror("OPEN_SESSION");
		return -1;
	}
	if (sess.arg.ret) {
		fprintf(stderr, "Open session failed: 0x%x\n", sess.arg.ret);
		return -1;
	}

	*session = sess.arg.session;
	return 0;
}

static void close_session(int fd, uint32_t session)
{
	struct tee_ioctl_close_session_arg carg;

	memset(&carg, 0, sizeof(carg));
	carg.session = session;
	ioctl(fd, TEE_IOC_CLOSE_SESSION, &carg);
}

static int do_ping(int fd, uint32_t session)
{
	struct tee_ioctl_param params[4];

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = 42;
	params[0].b = 100;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_PING, params))
		return -1;


	printf("PING OK: a=%llu b=%llu\n",
	       (unsigned long long)params[1].a,
	       (unsigned long long)params[1].b);
	return 0;
}

static const char *mode_str(uint32_t mode)
{
	switch (mode) {
	case 0:
		return "uninitialized";
	case 1:
		return "master";
	case 2:
		return "slave";
	default:
		return "unknown";
	}
}

static int do_status(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, packed, mode, fifo;
	int bus_busy;

	if (argc < 3) {
		fprintf(stderr, "Usage: status <inst>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	if (invoke(fd, session, CMD_STATUS, params))
		return -1;

	packed = (uint32_t)params[1].a;
	mode = packed & 0x03;
	bus_busy = !!(packed & 0x04);
	fifo = (packed >> 8) & 0xff;

	printf("I2C%u: %s\n", instance, mode_str(mode));

	if (mode == 0) /* uninitialized */
		return 0;

	printf("  Speed:	   %llu kHz\n",
	       (unsigned long long)params[1].b);
	printf("  FIFO size:	   %u bytes\n", fifo);
	printf("  Bus busy:	   %s\n", bus_busy ? "yes" : "no");
	printf("  Inter-msg delay: %llu us\n",
	       (unsigned long long)params[2].b);

	if (mode == 2) { /* slave */
		printf("  Slave address:   0x%02llx\n",
		       (unsigned long long)params[2].a);
		printf("  Write count:	   %llu\n",
		       (unsigned long long)params[3].a);
		printf("  Read count:	   %llu\n",
		       (unsigned long long)params[3].b);
	}

	return 0;
}

static int do_init(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, speed_khz, delay_us;

	if (argc < 3) {
		fprintf(stderr,
			"Usage: init <inst> [speed_khz] [inter_msg_delay_us]\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	speed_khz = (argc > 3) ? strtoul(argv[3], NULL, 0) : 400;
	delay_us = (argc > 4) ? strtoul(argv[4], NULL, 0) : 0;

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[0].b = speed_khz;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[1].a = delay_us;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_INIT, params))
		return -1;

	printf("Init OK: instance=%u speed=%ukHz delay=%uus\n",
	       instance, speed_khz, delay_us);
	return 0;
}

static int do_deinit(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance;

	if (argc < 3) {
		fprintf(stderr, "Usage: deinit <inst>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_DEINIT, params))
		return -1;

	printf("Deinit OK: instance=%u\n", instance);
	return 0;
}

static int do_read(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, addr, reg, len;

	if (argc < 6) {
		fprintf(stderr, "Usage: read <inst> <addr> <reg> <len>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	addr = strtoul(argv[3], NULL, 0);
	reg = strtoul(argv[4], NULL, 0);
	len = strtoul(argv[5], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[0].b = addr;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[1].a = reg;
	params[1].b = len;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_SIMPLE_READ, params))
		return -1;

	printf("Read OK: data=0x%08llx len=%llu\n",
	       (unsigned long long)params[2].a,
	       (unsigned long long)params[2].b);

	/* Print individual bytes */
	uint32_t packed = (uint32_t)params[2].a;
	uint32_t actual = (uint32_t)params[2].b;

	for (uint32_t i = 0; i < actual; i++)
		printf("  [%u] = 0x%02x\n", i,
		       (packed >> (i * 8)) & 0xff);

	return 0;
}

static int do_write(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, addr, reg, data_packed, len;

	if (argc < 7) {
		fprintf(stderr,
			"Usage: write <inst> <addr> <reg> <data> <len>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	addr = strtoul(argv[3], NULL, 0);
	reg = strtoul(argv[4], NULL, 0);
	data_packed = strtoul(argv[5], NULL, 0);
	len = strtoul(argv[6], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[0].b = addr;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[1].a = reg;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[2].a = data_packed;
	params[2].b = len;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_SIMPLE_WRITE, params))
		return -1;

	printf("Write OK: inst=%u addr=0x%02x reg=0x%02x data=0x%x len=%u\n",
	       instance, addr, reg, data_packed, len);
	return 0;
}

static int do_slave_register(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, addr;

	if (argc < 4) {
		fprintf(stderr, "Usage: slave_register <inst> <addr>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	addr = strtoul(argv[3], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[0].b = addr;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_SLAVE_REGISTER, params))
		return -1;

	printf("Slave registered: inst=%u addr=0x%02x\n", instance, addr);
	return 0;
}

static int do_slave_unregister(int fd, uint32_t session, int argc,
			       char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance;

	if (argc < 3) {
		fprintf(stderr, "Usage: slave_unregister <inst>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_SLAVE_UNREGISTER, params))
		return -1;

	printf("Slave unregistered: inst=%u\n", instance);
	return 0;
}

static int do_slave_set_regs(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, start, len, lo, hi;

	if (argc < 6) {
		fprintf(stderr,
			"Usage: slave_set_regs <inst> <start> <len> <lo> [hi]\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	start = strtoul(argv[3], NULL, 0);
	len = strtoul(argv[4], NULL, 0);
	lo = strtoul(argv[5], NULL, 0);
	hi = (argc > 6) ? strtoul(argv[6], NULL, 0) : 0;

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[1].a = start;
	params[1].b = len;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[2].a = lo;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[3].a = hi;

	if (invoke(fd, session, CMD_SLAVE_SET_REGS, params))
		return -1;

	printf("Slave regs set: inst=%u start=%u len=%u lo=0x%x hi=0x%x\n",
	       instance, start, len, lo, hi);
	return 0;
}

static int do_slave_get_regs(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, start, len;

	if (argc < 5) {
		fprintf(stderr,
			"Usage: slave_get_regs <inst> <start> <len>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	start = strtoul(argv[3], NULL, 0);
	len = strtoul(argv[4], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[1].a = start;
	params[1].b = len;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	if (invoke(fd, session, CMD_SLAVE_GET_REGS, params))
		return -1;

	uint32_t lo = (uint32_t)params[2].a;
	uint32_t hi = (uint32_t)params[3].a;

	printf("Slave regs: lo=0x%08x hi=0x%08x\n", lo, hi);

	for (uint32_t i = 0; i < len && i < 4; i++)
		printf("  [%u] = 0x%02x\n", start + i,
		       (lo >> (i * 8)) & 0xff);
	for (uint32_t i = 4; i < len; i++)
		printf("  [%u] = 0x%02x\n", start + i,
		       (hi >> ((i - 4) * 8)) & 0xff);

	return 0;
}

static int do_slave_poll(int fd, uint32_t session, int argc, char *argv[])
{
	struct tee_ioctl_param params[4];
	uint32_t instance, timeout_ms;

	if (argc < 4) {
		fprintf(stderr, "Usage: slave_poll <inst> <timeout_ms>\n");
		return -1;
	}

	instance = strtoul(argv[2], NULL, 0);
	timeout_ms = strtoul(argv[3], NULL, 0);

	memset(params, 0, sizeof(params));
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	params[0].a = instance;
	params[0].b = timeout_ms;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	if (invoke(fd, session, CMD_SLAVE_POLL, params))
		return -1;

	printf("Slave poll: writes=%llu reads=%llu\n",
	       (unsigned long long)params[1].a,
	       (unsigned long long)params[1].b);
	return 0;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s <command> [args...]\n"
		"\n"
		"Commands:\n"
		"  ping\n"
		"  status <inst>\n"
		"  init <inst> [speed_khz] [inter_msg_delay_us]\n"
		"  deinit <inst>\n"
		"  read <inst> <addr> <reg> <len>\n"
		"  write <inst> <addr> <reg> <data> <len>\n"
		"  slave_register <inst> <addr>\n"
		"  slave_unregister <inst>\n"
		"  slave_set_regs <inst> <start> <len> <lo> [hi]\n"
		"  slave_get_regs <inst> <start> <len>\n"
		"  slave_poll <inst> <timeout_ms>\n",
		prog);
}

int main(int argc, char *argv[])
{
	int fd;
	uint32_t session;
	int ret = -1;

	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	fd = open(TEE_DEVICE, O_RDWR);
	if (fd < 0) {
		perror("open " TEE_DEVICE);
		return 1;
	}

	if (open_session(fd, &session)) {
		close(fd);
		return 1;
	}

	if (!strcmp(argv[1], "ping"))
		ret = do_ping(fd, session);
	else if (!strcmp(argv[1], "status"))
		ret = do_status(fd, session, argc, argv);
	else if (!strcmp(argv[1], "init"))
		ret = do_init(fd, session, argc, argv);
	else if (!strcmp(argv[1], "deinit"))
		ret = do_deinit(fd, session, argc, argv);
	else if (!strcmp(argv[1], "read"))
		ret = do_read(fd, session, argc, argv);
	else if (!strcmp(argv[1], "write"))
		ret = do_write(fd, session, argc, argv);
	else if (!strcmp(argv[1], "slave_register"))
		ret = do_slave_register(fd, session, argc, argv);
	else if (!strcmp(argv[1], "slave_unregister"))
		ret = do_slave_unregister(fd, session, argc, argv);
	else if (!strcmp(argv[1], "slave_set_regs"))
		ret = do_slave_set_regs(fd, session, argc, argv);
	else if (!strcmp(argv[1], "slave_get_regs"))
		ret = do_slave_get_regs(fd, session, argc, argv);
	else if (!strcmp(argv[1], "slave_poll"))
		ret = do_slave_poll(fd, session, argc, argv);
	else {
		fprintf(stderr, "Unknown command: %s\n", argv[1]);
		usage(argv[0]);
	}

	close_session(fd, session);
	close(fd);

	return ret ? 1 : 0;
}
