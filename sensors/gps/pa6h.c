/*
 * Phoenix-RTOS
 *
 * Driver for GPS module PA6H
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"


typedef struct {
	sensor_event_t evtGps;
	char stack[512] __attribute__((aligned(8)));
} pa6h_ctx_t;


static inline time_t pa6h_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static void pa6h_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	pa6h_ctx_t *ctx = info->ctx;

	while (1) {
		sleep(1);
		/* TODO: set interval and publish data to sensor manager */

		/* Sample data */
		ctx->evtGps.gps.lat = 1;
		ctx->evtGps.gps.lon = 2;
		ctx->evtGps.gps.hdop = 3;
		ctx->evtGps.gps.vdop = 4;
		ctx->evtGps.gps.alt = 5;
		ctx->evtGps.gps.altEllipsoid = 6;
		ctx->evtGps.gps.groundSpeed = 7;
		ctx->evtGps.gps.velNorth = 8;
		ctx->evtGps.gps.velEast = 9;
		ctx->evtGps.gps.velDown = 10;
		ctx->evtGps.gps.eph = 11;
		ctx->evtGps.gps.epv = 12;
		ctx->evtGps.gps.heading = 13;
		ctx->evtGps.gps.headingOffs = 14;
		ctx->evtGps.gps.headingAccur = 15;
		ctx->evtGps.gps.satsNb = 16;

		ctx->evtGps.timestamp = pa6h_timeMsGet();
		sensors_publish(info->id, &ctx->evtGps);
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
	ctx->evtGps.gps.devId = info->id; //now its random ??

	info->ctx = ctx;

	err = beginthread(pa6h_threadPublish, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("pa6h: launched sensor\n");
	}

	return err;
}


static int pa6h_alloc(sensor_info_t *info, const char *args)
{
	info->types = SENSOR_TYPE_GPS;

	/* TODO: parse arguments and initialize device */

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
