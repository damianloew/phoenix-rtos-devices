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
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"
#include "nmea.h"


/* Specified by the module's documentation */
#define UPDATE_RATE_MS 1000


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	char buff[1024];
	char stack[512] __attribute__((aligned(8)));
} pa6h_ctx_t;


int pa6h_update(nmea_t *message, pa6h_ctx_t *ctx)
{
	switch (message->type) {
		case nmea_gga:
			ctx->evtGps.gps.lat = message->msg.gga.lat * 1e7;
			ctx->evtGps.gps.lon = message->msg.gga.lon * 1e7;
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
			ctx->evtGps.gps.fix = message->msg.gga.fix;
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

	return 0;
}


static void pa6h_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	pa6h_ctx_t *ctx = info->ctx;
	char *start, **lines;
	int nlines = 0, nbytes = 0, ret = 0;
	nmea_t message;

	while (1) {
		memset(ctx->buff, 0, sizeof(ctx->buff));
		usleep(1000 * UPDATE_RATE_MS);

		nbytes = read(ctx->filedes, ctx->buff, sizeof(ctx->buff) - 1);

		if (nbytes > 0) {
			start = ctx->buff;

			nlines = nmea_countlines(&start);
			lines = nmea_getlines(&start, nlines);

			for (int i = 0; i < nlines; i++) {
				if (nmea_assertChecksum(lines[i]) == EOK) {
					ret = nmea_interpreter(lines[i], &message);
					if (ret != nmea_broken && ret != nmea_unknown) {
						pa6h_update(&message, ctx);
					}
				}
			}
			sensors_publish(info->id, &ctx->evtGps);
		}
	}
}


static int pa6h_start(sensor_info_t *info)
{
	int err;
	pa6h_ctx_t *ctx = info->ctx;

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;

	err = beginthread(pa6h_threadPublish, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("pa6h: launched sensor\n");
	}

	return err;
}


static int pa6h_parse(const char *args, const char **path)
{
	int err = EOK;

	if (!(args == NULL || strchr(args, ':'))) {
		*path = args;
	}
	else {
		fprintf(stderr, "pa6h: Wrong arguments\n"
						"Please specify the path to source device instance, for example: /dev/uart0\n");
		err = -EINVAL;
	}

	return err;
}


static int pa6h_alloc(sensor_info_t *info, const char *args)
{
	int cnt = 0, err = EOK;
	const char *path;
	pa6h_ctx_t *ctx = malloc(sizeof(pa6h_ctx_t));

	if (ctx == NULL) {
		return -ENOMEM;
	}

	info->types = SENSOR_TYPE_GPS;

	err = pa6h_parse(args, &path);

	if (err == EOK) {
		while ((ctx->filedes = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK)) < 0) {
			usleep(10 * 1000);
			cnt++;

			if (cnt > 10000) {
				fprintf(stderr, "Can't open %s: %s\n", path, strerror(errno));
				err = -errno;
				free(ctx);
			}
		}
		if (ctx->filedes > 0) {
			info->ctx = ctx;
		}
	}

	return err;
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
