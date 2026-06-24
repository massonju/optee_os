global-incdirs-y += .
srcs-y += main.c
srcs-$(CFG_OMAP_I2C) += k3_i2c.c
subdirs-y += drivers
