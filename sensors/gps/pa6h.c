/*
 * Phoenix-RTOS
 *
 * Driver for GPS module PA6H
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"
#include "nmea.h"


typedef struct {
	sensor_event_t evtGps;
	handle_t lock;
	FILE *srcdevfile;
	char measureStack[2048] __attribute__((aligned(8)));
	char publishStack[512] __attribute__((aligned(8)));
} pa6h_ctx_t;


static void pa6h_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	pa6h_ctx_t *ctx = info->ctx;
	while (1) {
		usleep(500000);
		mutexLock(ctx->lock);
		sensors_publish(info->id, &ctx->evtGps);
		mutexUnlock(ctx->lock);
	}
}


int pa6h_update(nmea_t *message, pa6h_ctx_t *ctx)
{
	mutexLock(ctx->lock);
	switch (message->type) {
		case nmea_gga:
			ctx->evtGps.gps.lat = message->msg.gga.lat * 1e7;
			ctx->evtGps.gps.lon = message->msg.gga.lon * 1e7;
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
			// ctx->evtGps.gps.fix = message->msg.gga.fix;
			ctx->evtGps.gps.alt = message->msg.gga.h_asl * 1e3;
			ctx->evtGps.gps.altEllipsoid = message->msg.gga.h_wgs * 1e3;
			ctx->evtGps.gps.satsNb = message->msg.gga.sats;
			break;

		case nmea_gsa:
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gsa.hdop * 1e2);
			ctx->evtGps.gps.vdop = (unsigned int)(message->msg.gsa.vdop * 1e2);
			break;

		case nmea_rmc:
			break;

		case nmea_vtg:
			ctx->evtGps.gps.heading = message->msg.vtg.track * 0.017453 * 1e3;            /* degrees -> milliradians */
			ctx->evtGps.gps.groundSpeed = message->msg.vtg.speed_kmh * 1e6 * 0.000277778; /* kmh->mm/s */
			ctx->evtGps.gps.velNorth = cos(message->msg.vtg.track * 0.017453) * ctx->evtGps.gps.groundSpeed;
			ctx->evtGps.gps.velEast = sin(message->msg.vtg.track * 0.017453) * ctx->evtGps.gps.groundSpeed;
			break;

		default:
			break;
	}
	gettime(&(ctx->evtGps.timestamp), NULL);
	mutexUnlock(ctx->lock);

	return 0;
}


static void pa6h_threadMeasure(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	pa6h_ctx_t *ctx = info->ctx;
	char buf[1024], *start, **lines;
	int n, ret;
	nmea_t message;

	while (1) {
		memset(buf, 0, sizeof(buf));
		usleep(500000);

		fread(buf, 1, sizeof(buf) - 1, ctx->srcdevfile);
		start = buf;

		n = nmea_countlines(&start);
		lines = nmea_getlines(&start, n);

		for (int i = 0; i < n; i++) {
			if (nmea_assertChecksum(lines[i]) == EOK) {
				ret = nmea_interpreter(lines[i], &message);
				if (ret != nmea_broken && ret != nmea_unknown) {
					pa6h_update(&message, ctx);
				}
			}
		}
	}
}


static int pa6h_start(sensor_info_t *info)
{
	int err;
	pa6h_ctx_t *ctx = malloc(sizeof(pa6h_ctx_t));

	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;
	ctx->srcdevfile = fopen(info->srcdev, "r");

	info->ctx = ctx;

	if (ctx->srcdevfile != NULL) {
		mutexCreate(&(ctx->lock));
		err = beginthread(pa6h_threadMeasure, 4, ctx->measureStack, sizeof(ctx->measureStack), info);
		if (err < 0) {
			free(ctx);
		}
		else {
			err = beginthread(pa6h_threadPublish, 4, ctx->publishStack, sizeof(ctx->publishStack), info);
			if (err < 0) {
				free(ctx);
			}
			else {
				printf("pa6h: launched sensor\n");
			}
		}
	}
	else {
		fprintf(stderr, "Can't open %s: %s\n", info->srcdev, strerror(errno));
		err = errno;
	}

	return err;
}


static int pa6h_alloc(sensor_info_t *info, const char *args)
{
	info->types = SENSOR_TYPE_GPS;
	/* TODO: set proper id */
	info->id = 1;
	/* TODO: parse some additional arguments if needed */
	if (args) {
		info->srcdev = args;
	}
	else {
		/* default source interface */
		info->srcdev = "/dev/uart0";
	}

	return EOK;
}


void __attribute__((constructor)) pa6h_register(void)
{
	static sensor_drv_t sensor = {
		.name = "pa6h",
		.alloc = pa6h_alloc,
		.start = pa6h_start
	};

	sensors_register(&sensor);
}
