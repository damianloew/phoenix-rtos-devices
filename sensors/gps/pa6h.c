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


/* TODO: remove */
void print_gps_data(gps_data_t *data)
{
	printf("lat/lon | hdop:\t%d/%d | %d\n", data->lat, data->lon, data->hdop);
	printf("asl/wgs | vdop:\t%d/%d | %d\n", data->alt, data->altEllipsoid, data->vdop);
	printf("kmh/kmhN/kmhE:\t%d/%d/%d\n", data->groundSpeed, data->velNorth, data->velEast);
	printf("hdop/vdop/sats:\t%d/%d/%d\n", data->hdop, data->vdop, data->satsNb);
	printf("\n");
}


static int pa6h_getlines(char **buf)
{
	int n = 0;
	char *start, *curr;

	start = strchr(*buf, '$');
	if (start == NULL) {
		return 0;
	}

	curr = start;
	do {
		curr = strchr(curr, '$');
		if (curr != NULL) {
			curr = strchr(curr, '*');
		}

		n++;
	} while (curr != NULL);
	n--;

	if (n > 0) {
		*buf = start;
	}

	return n;
}


int pa6h_update(nmea_t *message, pa6h_ctx_t *ctx)
{
	mutexLock(ctx->lock);
	switch (message->type) {
		case nmea_gga:
			ctx->evtGps.gps.lat = message->msg.gga.lat * 1e7;
			ctx->evtGps.gps.lon = message->msg.gga.lon * 1e7;
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
			//ctx->evtGps.gps.fix = message->msg.gga.fix;
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
	char buf[1024], *start;
	int n, ret;
	nmea_t message;

	while (1) {
		memset(buf, 0, sizeof(buf));
		usleep(500000);

		fread(buf, 1, sizeof(buf), ctx->srcdevfile);
		start = buf;
		n = pa6h_getlines(&start);
		while (n > 0) {
			ret = nmea_interpreter(start, &message);
			if (ret != nmea_broken && ret != nmea_unknown) {
				pa6h_update(&message, ctx);
			}
			n--;
			start = strchr(start + 1, '$');
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
