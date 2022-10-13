/*
 * Phoenix-RTOS
 *
 * nrf91 TTY driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <posix/utils.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <libtty.h>
// #include <libuart.h>

#include "nrf91-multi.h"
#include "config.h"
#include "common.h"
#include "gpio.h"
#include "tty.h"
// #include "uart.h"
// #include "rcc.h"

#define TTY0_POS 0
#define TTY1_POS (TTY0_POS + TTY0)
#define TTY2_POS (TTY1_POS + TTY1)
#define TTY3_POS (TTY2_POS + TTY2)

#define TTY_CNT (TTY0 + TTY1 + TTY2 + TTY3)

#define THREAD_POOL 3
#define THREAD_STACKSZ 768
#define THREAD_PRIO 1

#if TTY_CNT != 0
typedef struct {
	char stack[512] __attribute__ ((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	volatile char *tx_dma;
	volatile char *rx_dma;

	int bits;
	int parity;
	int baud;
	unsigned int cnt;

	handle_t cond;
	handle_t inth;
	handle_t irqlock;

	libtty_common_t tty_common;

	volatile unsigned char rxbuff;
	volatile int rxready;
} tty_ctx_t;


static struct {
	unsigned char poolstack[THREAD_POOL][THREAD_STACKSZ] __attribute__((aligned(8)));
	tty_ctx_t ctx[TTY_CNT];

	unsigned int port;
} uart_common;

static unsigned int common_port;

static const int uartConfig[] = { TTY0, TTY1, TTY2, TTY3 };


static const int uartPos[] = { TTY0_POS, TTY1_POS, TTY2_POS, TTY3_POS };

/* IT'S VISIBLE FROM KERNEL ???*/
// enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, uarte_flushrx = 11,
// uarte_events_cts = 64, uarte_events_rxdrdy = 66, uarte_events_endrx = 68, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_rxto = 81, uarte_events_txstarted = 84, 
// uarte_inten = 192, uarte_intenset, uarte_intenclr, uarte_errorsrc = 288, uarte_enable = 320, 
// uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
// uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
// uarte_config = 347 };


enum { tty_parnone = 0, tty_pareven, tty_parodd };


static inline int tty_txready(tty_ctx_t *ctx)
{
	//not working
	/* clear txdrdy flag if there endtx event has been occurred */
	// if (*(ctx->base + uarte_events_endtx)) {
	// 	*(ctx->base + uarte_events_txdrdy) = 0u;
	// }
	int ret = *(ctx->base + uarte_events_txdrdy);
	*(ctx->base + uarte_events_txdrdy) = 0u;
	
	/* if endtx event */
//its 0 checktxdrdy reg!!
	return ret; //*(ctx->base + uarte_events_txdrdy); //there was endtx before 
}


static int tty_irqHandler(unsigned int n, void *arg)
{

	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	if (*(ctx->base + uarte_events_rxdrdy)) {
		/* clear rxdrdy event flag */
		*(ctx->base + uarte_events_rxdrdy) = 0u;

		/* I think that because of rxready there cnt isn't necessary */
		ctx->rxbuff = ctx->rx_dma[ctx->cnt];
		ctx->cnt++;
		ctx->rxready = 1;
	}

	// if (tty_txready(ctx)) {
	// 	/* clear endtx event flag */
	// 	// *(ctx->base + uarte_events_endtx) = 0u;
	// 	/* disable endtx interrupt but don't clear flag */
	// 	*(ctx->base + uarte_intenclr) = 0x100;
	// }

	if (*(ctx->base + uarte_events_endtx) == 1u) {
		/* disable endtx interrupt but don't clear flag */
		*(ctx->base + uarte_intenclr) = 0x100;
		// ctx->cnt = 0;
		// *(ctx->base + uarte_events_endtx) = 0u;
	}
				// while ( *(uart->base + uarte_events_endtx) != 1u )
				// 	;
				// *(uart->base + uarte_events_endtx) = 0u;

	/* previous implementation*/
	// if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
	// 	/* Clear overrun error bit */
	// 	*(ctx->base + icr) |= (1 << 3);

	// 	ctx->rxbuff = *(ctx->base + rdr);
	// 	ctx->rxready = 1;
	// }

	// if (tty_txready(ctx))
	// 	*(ctx->base + cr1) &= ~(1 << 7);

	// if (uart->base + uarte_events_endrx) {
	// 	/* clear endrx event flag */
	// 	*(uart->base + uarte_events_endrx) = 0u;
	// 	uart->cnt = 0;
	// 	*(uart->base + uarte_startrx) = 1u;
	// 	hal_cpuDataMemoryBarrier();
	// }

	return 1;
}

// static int tty_irqHandler(unsigned int n, void *arg)
// {
// 	tty_ctx_t *ctx = (tty_ctx_t *)arg;

// 	if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
// 		/* Clear overrun error bit */
// 		*(ctx->base + icr) |= (1 << 3);

// 		ctx->rxbuff = *(ctx->base + rdr);
// 		ctx->rxready = 1;
// 	}

// 	if (tty_txready(ctx))
// 		*(ctx->base + cr1) &= ~(1 << 7);

// 	return 1;
// }


// isn't data dbarrier needed here ???
/* assuming it's ok - implemetation from stm multi */
static void tty_irqthread(void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;
	int keptidle = 0;

	/* TODO add small TX and RX buffers that can be directly read / written in irq handlers */

	while (1) {
		mutexLock(ctx->irqlock);
		// it does not go furtherit waits for condBroadcast?
		while ((!ctx->rxready && !((libtty_txready(&ctx->tty_common) || keptidle))) || !(*(ctx->base + uarte_enable) & 0x08))
			condWait(ctx->cond, ctx->irqlock, 0);
		mutexUnlock(ctx->irqlock);

		if (ctx->rxready) {
			libtty_putchar(&ctx->tty_common, ctx->rxbuff, NULL);
			ctx->rxready = 0;
		}

		if (libtty_txready(&ctx->tty_common)) {
			if (tty_txready(ctx)) {
				if (!keptidle) {
					keptidle = 1;
					keepidle(1);
				}

				ctx->tx_dma[0] = libtty_getchar(&ctx->tty_common, NULL);
				/* clear flag */
				*(ctx->base + uarte_events_endtx) = 0u;
				ctx->cnt = 0;
				*(ctx->base + uarte_startrx) = 1u;
				/* enable endtx interrupt */
				*(ctx->base + uarte_intenset) = 0x100;
				*(ctx->base + uarte_starttx) = 1u;

				/* not sure if should I wait for sending it */
				// while ( *(uart->base + uarte_events_txstarted) != 1u )
				// 	;
				// *(uart->base + uarte_events_txstarted) = 0u;

				// while ( *(uart->base + uarte_events_endtx) != 1u )
				// 	;
				// *(uart->base + uarte_events_endtx) = 0u;

				/* enabling tx interrupt, why????*/
				// *(ctx->base + cr1) |= (1 << 7);
			}
		}
		else if (keptidle) {
			keptidle = 0;
			keepidle(0);
		}
	}
}


static void tty_signalTxReady(void *ctx)
{
	condSignal(((tty_ctx_t *)ctx)->cond);
}


static int _tty_configure(tty_ctx_t *ctx, char bits, char parity, char enable)
{
	int err = EOK;
	// unsigned int tcr1 = 0;
	// char tbits = bits;

	/* If target uart instance is enabled - disable it before configuration */
	if (*(ctx->base + uarte_enable) & 0x08) {
		*(ctx->base + uarte_enable) = 0u;
		dataBarier();
	}
	/* TODO: add pins configuartion and selecting them, now it's done in plo
	I have to ask about it coz can't find gpio configuration in tty/uart/spi drivers for stm */
	//uart_configPins(minor);
	/* Select pins */
	// *(ctx->base + uarte_psel_txd) = uartInfo[minor].txpin;
	// *(ctx->base + uarte_psel_rxd) = uartInfo[minor].rxpin;
	// *(ctx->base + uarte_psel_rts) = uartInfo[minor].rtspin;
	// *(ctx->base + uarte_psel_cts) = uartInfo[minor].ctspin;

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	/* TODO: add configuration based on args */
	*(ctx->base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers */
	*(ctx->base + uarte_txd_maxcnt) = 1;
	*(ctx->base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(ctx->base + uarte_txd_ptr) = (unsigned int)ctx->tx_dma;
	*(ctx->base + uarte_rxd_ptr) = (unsigned int)ctx->rx_dma;

	/* clear rxdrdy flag */
	*(ctx->base + uarte_events_rxdrdy) = 0u;

	/* Disable all uart interrupts */
	*(ctx->base + uarte_intenclr) = 0xFFFFFFFF;
	// on stm it's when there is some data on rxdrdy
	// here rxdrdy and rxto is set - i dont want interrupt when there is timoeut
	/* Enable rxdrdy interruts */
	*(ctx->base + uarte_intenset) = 0x4;
	dataBarier();

	/* Enable uarte instance */
	*(ctx->base + uarte_enable) = 0x8;
	dataBarier();
	ctx->cnt = 0;
	*(ctx->base + uarte_startrx) = 1u;
	ctx->enabled = 1;
	dataBarier();

	return err;
}


static void tty_setCflag(void *uart, tcflag_t *cflag)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	char bits, parity = tty_parnone;

	if ((*cflag & CSIZE) == CS6)
		bits = 6;
	else if ((*cflag & CSIZE) == CS7)
		bits = 7;
	else
		bits = 8;

	if (*cflag & PARENB) {
		if (*cflag & PARODD)
			parity = tty_parodd;
		else
			parity = tty_pareven;
	}

	if (bits != ctx->bits || parity != ctx->parity) {
		_tty_configure(ctx, bits, parity, 1);
		condSignal(ctx->cond);
	}

	ctx->bits = bits;
	ctx->parity = parity;
}


static void tty_setBaudrate(void *uart, speed_t baud)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	int baudr = libtty_baudrate_to_int(baud);

	if (ctx->baud != baudr) {
		switch (baudr) {
		case 9600:
			*(ctx->base + uarte_baudrate) = baud_9600;
			break;
		case 115200:
		default:
			*(ctx->base + uarte_baudrate) = baud_115200;
		}
		dataBarier();

		ctx->enabled = 1;
		condSignal(ctx->cond);
	}

	ctx->baud = baudr;
}


static tty_ctx_t *tty_getCtx(id_t id)
{
	tty_ctx_t *ctx = NULL;

	if (!id) {
		id = uart0 + UART_CONSOLE;
	}

	if (id >= uart0 && id <= uart3) {
		ctx = &uart_common.ctx[uartPos[0]];
	}

	return ctx;
}


static void tty_thread(void *arg)
{
	msg_t msg;
	unsigned long rid;
	tty_ctx_t *ctx;
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	id_t id;
	// uart_common.port = 0;

	while (1) {
		while (msgRecv(common_port, &msg, &rid) < 0)
			;

		priority(msg.priority);

		switch (msg.type) {
		case mtOpen:
		case mtClose:
			if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}

			msg.o.io.err = EOK;
			break;

		case mtWrite:
			if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}
			msg.o.io.err = libtty_write(&ctx->tty_common, msg.i.data, msg.i.size, msg.i.io.mode);
			break;

		case mtRead:
			if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}
			msg.o.io.err = libtty_read(&ctx->tty_common, msg.o.data, msg.o.size, msg.i.io.mode);
			break;

		case mtGetAttr:
			if ((msg.i.attr.type != atPollStatus) || ((ctx = tty_getCtx(msg.i.attr.oid.id)) == NULL)) {
				msg.o.attr.err = -EINVAL;
				break;
			}
			msg.o.attr.val = libtty_poll_status(&ctx->tty_common);
			msg.o.attr.err = EOK;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(&msg, &request, &id);
			if ((ctx = tty_getCtx(id)) == NULL) {
				err = -EINVAL;
			}
			else {
				pid = ioctl_getSenderPid(&msg);
				err = libtty_ioctl(&ctx->tty_common, pid, request, in_data, &out_data);
			}
			ioctl_setResponse(&msg, request, err, out_data);
			break;
		}

		msgRespond(uart_common.port, &msg, rid);

		priority(THREAD_PRIO);
	}
}
#endif


void tty_log(const char *str)
{
#if CONSOLE_IS_TTY
	//here  it goes
	libtty_write(&tty_getCtx(0)->tty_common, str, strlen(str), 0);
#endif
}


int tty_init(unsigned int *port)
{
#if TTY_CNT != 0
	unsigned int uart, i;
	char fname[] = "uartx";
	// char str[180];
	int test = 2;
	oid_t oid;
	libtty_callbacks_t callbacks;
	tty_ctx_t *ctx;
	// static const struct {
	// 	volatile uint32_t *base;
	// 	int dev;
	// 	unsigned irq;
	// } info[] = {
	// 	{ (void *)0x40013800, pctl_usart1, usart1_irq },
	// 	{ (void *)0x40004400, pctl_usart2, usart2_irq },
	// 	{ (void *)0x40004800, pctl_usart3, usart3_irq },
	// 	{ (void *)0x40004c00, pctl_uart4, uart4_irq },
	// 	{ (void *)0x40005000, pctl_uart5, uart5_irq },
	// };

static const struct {
	volatile uint32_t *base;
	unsigned int irq;
	volatile char *tx_dma;
	volatile char *rx_dma;
} info[] = {
	{ (void *)UART0_BASE, UART0_IRQ, (volatile char *)UART0_TX_DMA, (volatile char *)UART0_RX_DMA },
	{ (void *)UART1_BASE, UART1_IRQ, (volatile char *)UART1_TX_DMA, (volatile char *)UART1_RX_DMA },
	{ (void *)UART2_BASE, UART2_IRQ, (volatile char *)UART2_TX_DMA, (volatile char *)UART2_RX_DMA },
	{ (void *)UART3_BASE, UART3_IRQ, (volatile char *)UART3_TX_DMA, (volatile char *)UART3_RX_DMA }
};

	// sprintf(str, "&uart_common.port = %p\n", &uart_common.port);
	// debug(str);

	// sprintf(str, "TTY0 = %d, TTY1 = %d, TTY2 = %d, TTY3 = %d, &uartConfig= %p, uartConfig[0] = %d, uartConfig[1] = %d, uartConfig[2] = %d, uartConfig[3] = %d\n", TTY0, TTY1, TTY2, TTY3, uartConfig, uartConfig[0], uartConfig[1], uartConfig[2], uartConfig[3]);
	// // sprintf(str, "&uartConfig= %p, uartConfig[0] = %d\n", uartConfig, uartConfig[0]);
	// debug(str);

	test=4;
	if (port == NULL)
		portCreate(&uart_common.port);
	else
		uart_common.port = *port;

	oid.port = uart_common.port;
	common_port = 0;
#if CONSOLE_IS_TTY
	oid.id = 0;
	//there program stops
	create_dev(&oid, "tty");
#endif

	for (uart = uart0; uart <= uart3; uart++) {
		if (!uartConfig[uart])
			continue;


		ctx = &uart_common.ctx[uart];

		// devClk(info[uart - usart1].dev, 1);

		callbacks.arg = ctx;
		callbacks.set_baudrate = tty_setBaudrate;
		callbacks.set_cflag = tty_setCflag;
		callbacks.signal_txready = tty_signalTxReady;

		if (libtty_init(&ctx->tty_common, &callbacks, 512) < 0) {
			return -1;
		}

		mutexCreate(&ctx->irqlock);
		condCreate(&ctx->cond);

		ctx->base = info[uart - uart0].base;
		ctx->tx_dma = info[uart - uart0].tx_dma;
		ctx->rx_dma = info[uart - uart0].rx_dma;
		ctx->rxready = 0;
		ctx->bits = -1;
		ctx->parity = -1;
		ctx->baud = -1;

		/* Set up UART to 115200,8,n,1 16-bit oversampling */
		_tty_configure(ctx, 8, tty_parnone, 1);
		/* baud rate should be set earlier, before enabling uart */
		tty_setBaudrate(ctx, B115200);
		//here

		/* don't see info irq*/
		interrupt(info[uart - uart0].irq, tty_irqHandler, (void *)ctx, ctx->cond, NULL);

		/* is stack initilized*/
		beginthread(tty_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);

		ctx->enabled = 1;

		fname[sizeof(fname) - 2] = '0' + uart - uart0;
		oid.id = uart - uart0 + 1;
		//endless loop
		create_dev(&oid, fname);
	}

	for (i = 0; i < THREAD_POOL; ++i)
		beginthread(tty_thread, THREAD_PRIO, uart_common.poolstack[i], sizeof(uart_common.poolstack[i]), (void *)i);

	return EOK;
#else
	return 0;
#endif
}
