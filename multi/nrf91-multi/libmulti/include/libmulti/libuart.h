/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 UART driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBUART_H_
#define _LIBUART_H_

#include <sys/threads.h>

typedef struct {
	volatile unsigned int *base;
	unsigned int port;
	unsigned int baud;
	volatile int enabled;
	int bits;

	volatile char *volatile txbeg;
	volatile char *volatile txend;

	volatile char rxdfifo[64];
	volatile unsigned int rxdr;
	volatile unsigned int rxdw;
	volatile char *volatile rxbeg;
	volatile char *volatile rxend;
	volatile unsigned int *volatile read;

	handle_t rxlock;
	handle_t rxcond;
	handle_t txlock;
	handle_t txcond;
	handle_t lock;
} libuart_ctx;


enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, uarte_flushrx = 11,
uarte_events_cts = 64, uarte_events_ncts, uarte_events_rxdrdy, uarte_events_endrx = 68, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_rxto = 81, uarte_events_rxstarted = 83, uarte_events_txstarted, uarte_events_txstopped = 86,
uarte_inten = 192, uarte_intenset, uarte_intenclr, uarte_errorsrc = 288, uarte_enable = 320, 
uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
uarte_config = 347 };


enum { baud_9600 = 0x00275000, baud_115200 = 0x01D60000 };


enum { uart0 = 0, uart1, uart2, uart3 };


enum { uart_mnormal = 0, uart_mnblock };


enum { uart_parnone = 0, uart_pareven, uart_parodd };


int libuart_configure(libuart_ctx *ctx, char bits, char parity, unsigned int baud, char enable);


int libuart_write(libuart_ctx *ctx, const void *buff, unsigned int bufflen);


int libuart_read(libuart_ctx *ctx, void* buff, unsigned int count, char mode, unsigned int timeout);


int libuart_init(libuart_ctx *ctx, unsigned int uart);


#endif
