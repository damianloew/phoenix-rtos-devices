/*
 * Phoenix-RTOS
 *
 * STM32L4 multi driver main
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>

#include "common.h"

#include "adc.h"
#include "exti.h"
#include "flash.h"
#include "fs.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"
#include "rtc.h"
#include "spi.h"
#include "tty.h"
#include "uart.h"

#define THREADS_NO 3
#define THREADS_PRIORITY 1
#define STACKSZ 640


struct {
	char stack[THREADS_NO - 1][STACKSZ] __attribute__ ((aligned(8)));

	unsigned int port;
} common;


static void handleMsg(msg_t *msg)
{
	multi_i_t *imsg = (multi_i_t *)msg->i.raw;
	multi_o_t *omsg = (multi_o_t *)msg->o.raw;
	int err = EOK;
	unsigned int t;

	switch (imsg->type) {
		case i2c_get:
			err = i2c_read(imsg->i2c_msg.i2c, imsg->i2c_msg.addr, msg->o.data, msg->o.size);
			break;

		case i2c_getwreg:
			err = i2c_readReg(imsg->i2c_msg.i2c, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->o.data, msg->o.size);
			break;

		case i2c_set:
			err = i2c_write(imsg->i2c_msg.i2c, imsg->i2c_msg.addr, msg->i.data, msg->i.size);
			break;

		case i2c_setwreg:
			err = i2c_writeReg(imsg->i2c_msg.i2c, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->i.data, msg->i.size);
			break;

		case exti_def:
			err = exti_configure(imsg->exti_def.line, imsg->exti_def.mode, imsg->exti_def.edge);
			break;

		case exti_map:
			err = syscfg_mapexti(imsg->exti_map.line, imsg->exti_map.port);
			break;

		case flash_get:
			err = flash_readData(imsg->flash_addr, msg->o.data, msg->o.size);
			break;

		case flash_set:
			err = flash_writeData(imsg->flash_addr, msg->i.data, msg->i.size);
			break;

		case flash_switch:
			if (imsg->flash_magic == FLASH_REBOOT_MAGIC)
				err = flash_switchBanks();
			break;

		case flash_info:
			flash_getInfo(&omsg->flash_info);
			break;

		case rtc_setcal:
			rtc_setCalib(imsg->rtc_calib);
			break;

		case rtc_get:
			rtc_getTime(&omsg->rtc_timestamp);
			break;

		case rtc_set:
			rtc_setTime(&imsg->rtc_timestamp);
			break;

		case rtc_setalarm:
			rtc_setAlarm(&imsg->rtc_timestamp);
			break;

		case adc_get:
			omsg->adc_valmv = adc_conversion(imsg->adc_get.adcno, imsg->adc_get.channel);
			break;

		case spi_get:
			err = spi_transaction(imsg->spi_rw.spi, spi_dir_read, imsg->spi_rw.cmd, imsg->spi_rw.addr,
				imsg->spi_rw.flags, msg->o.data, NULL, msg->o.size);
			break;

		case spi_set:
			err = spi_transaction(imsg->spi_rw.spi, spi_dir_write, imsg->spi_rw.cmd, imsg->spi_rw.addr,
				imsg->spi_rw.flags, NULL, msg->i.data, msg->i.size);
			break;

		case spi_rw:
			err = spi_transaction(imsg->spi_rw.spi, spi_dir_readwrite, imsg->spi_rw.cmd, imsg->spi_rw.addr,
				imsg->spi_rw.flags, msg->o.data, msg->i.data, (msg->i.size > msg->o.size) ? msg->o.size : msg->i.size);
			break;

		case spi_def:
			err = spi_configure(imsg->spi_def.spi, imsg->spi_def.mode, imsg->spi_def.bdiv, imsg->spi_def.enable);
			break;

		case gpio_def:
			err = gpio_configPin(imsg->gpio_def.port, imsg->gpio_def.pin, imsg->gpio_def.mode,
				imsg->gpio_def.af, imsg->gpio_def.otype, imsg->gpio_def.ospeed, imsg->gpio_def.pupd);
			break;

		case gpio_get:
			err = gpio_getPort(imsg->gpio_get.port, &t);
			omsg->gpio_get = t;
			break;

		case gpio_set:
			err = gpio_setPort(imsg->gpio_set.port, imsg->gpio_set.mask, imsg->gpio_set.state);
			break;

		case uart_def:
			err = uart_configure(imsg->uart_def.uart, imsg->uart_def.bits, imsg->uart_def.parity,
				imsg->uart_def.baud, imsg->uart_def.enable);
			break;

		case uart_get:
			err = uart_read(imsg->uart_get.uart, msg->o.data, msg->o.size,
				imsg->uart_get.mode, imsg->uart_get.timeout);
			break;

		case uart_set:
			err = uart_write(imsg->uart_set.uart, msg->i.data, msg->i.size);
			break;

		default:
			err = -EINVAL;
	}

	omsg->err = err;
}


static ssize_t console_write(const char *str, size_t len, int mode)
{
#if CONSOLE_IS_TTY
	tty_log(str);
	return (ssize_t)len;
#else
	return uart_write(UART_CONSOLE - 1, str, len);
#endif
}


static ssize_t console_read(char *str, size_t bufflen, int mode)
{
#if CONSOLE_IS_TTY
	return -ENOSYS;
#else
	return uart_read(UART_CONSOLE - 1, str, bufflen, uart_mnormal, 0);
#endif
}


static void thread(void *arg)
{
	msg_t msg;
	unsigned long int rid;

	while (1) {
		while (msgRecv(common.port, &msg, &rid) < 0)
			;

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtRead:
				msg.o.io.err = console_read(msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				msg.o.io.err = console_write(msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtDevCtl:
				handleMsg(&msg);
				break;

			case mtCreate:
				msg.o.create.err = -EINVAL;
				break;

			case mtLookup:
				msg.o.lookup.err = -EINVAL;
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(common.port, &msg, rid);

		priority(THREADS_PRIORITY);
	}
}


int main(void)
{
	int i;
	oid_t oid;
	static const char welcome[] = "multidrv: Started\n";
#if CONSOLE_IS_TTY
	unsigned int ttyConsolePort;
#endif

	priority(THREADS_PRIORITY);

#if CONSOLE_IS_TTY
	portCreate(&ttyConsolePort);
#endif
	portCreate(&common.port);

#if BUILTIN_DUMMYFS
	fs_init();
#else
	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);
#endif

	rcc_init();
	exti_init();
#if CONSOLE_IS_TTY
	tty_init(&ttyConsolePort);
#else
	tty_init(NULL);
#endif
	gpio_init();
	spi_init();
	adc_init();
	rtc_init();
	flash_init();
	i2c_init();
	uart_init();

	portRegister(common.port, "/multi", &oid);

	console_write(welcome, sizeof(welcome) - 1, 0);

	for (i = 0; i < THREADS_NO - 1; ++i)
		beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i);

	thread((void *)i);

	return 0;
}
