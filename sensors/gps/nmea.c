/*
 * Phoenix-RTOS
 *
 * NMEA
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
#include <string.h>
#include <errno.h>
#include <math.h>

#include "nmea.h"


/* moves pointer to the comma character just before the n-th field of nmea message in str. Returns pointer new value or NULL if search for n-th field failed */
static char *nmea_nField(char **pointer, char *str, int n)
{
	for (int i = 0; i < n && str != NULL; i++) {
		str = (*(str + sizeof(char)) == ',') ? str + sizeof(char) : strchr(str + 1, ',');
	}
	*pointer = str;
	return *pointer;
}


static int nmea_parsegsa(char *str, nmea_t *out)
{
	char *p;
	nmea_gsa_t in;
	do {
		if (nmea_nField(&p, str, field_gsa_fix) == NULL) {
			break;
		}
		in.fix = strtoul(p + 1, NULL, 10);
		if (in.fix == 0 || in.fix > 3) {
			break;
		}

		if (nmea_nField(&p, str, field_gsa_pdop) == NULL) {
			break;
		}
		in.pdop = strtod(p + 1, NULL);

		if (nmea_nField(&p, str, field_gsa_hdop) == NULL) {
			break;
		}
		in.hdop = strtod(p + 1, NULL);

		if (nmea_nField(&p, str, field_gsa_vdop) == NULL) {
			break;
		}
		in.vdop = strtod(p + 1, NULL);

		out->msg.gsa = in;
		out->type = nmea_gsa;
		return nmea_gsa;

	} while (0);

	return nmea_broken;
}


static int nmea_parsevtg(char *str, nmea_t *out)
{
	char *p;
	nmea_vtg_t in;

	do {
		if (nmea_nField(&p, str, field_vtg_track) == NULL) {
			break;
		}
		in.track = strtoul(p + 1, NULL, 10);

		if (nmea_nField(&p, str, field_vtg_speedknots) == NULL) {
			break;
		}
		in.speed_knots = strtod(p + 1, NULL);

		if (nmea_nField(&p, str, field_vtg_speedkmh) == NULL) {
			break;
		}
		in.speed_kmh = strtod(p + 1, NULL);

		out->msg.vtg = in;
		out->type = nmea_vtg;
		return nmea_vtg;

	} while (0);

	return nmea_broken;
}


static int nmea_parsegga(char *str, nmea_t *out)
{
	char *p, *sign;
	nmea_gga_t in;

	do {
		/* latitude read */
		if (nmea_nField(&p, str, field_gga_lat) == NULL || nmea_nField(&sign, str, field_gga_lat + 1) == NULL) {
			break;
		}
		in.lat = strtod(p + 1, NULL);
		in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
		in.lat = (*(sign + 1) == 'S') ? -in.lat : in.lat;

		/* longitude read */
		if (nmea_nField(&p, str, field_gga_lon) == NULL || nmea_nField(&sign, str, field_gga_lon + 1) == NULL) {
			break;
		}
		in.lon = strtod(p + 1, NULL);
		in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
		in.lon = (*(sign + 1) == 'S') ? -in.lon : in.lon;

		/* GPS Quality indicator read */
		if (nmea_nField(&p, str, field_gga_fix) == NULL) {
			break;
		}
		in.fix = strtoul(p + 1, NULL, 10);
		if (in.fix == 0 || in.fix > 3) {
			break;
		}

		/* number of satellites in use read */
		if (nmea_nField(&p, str, field_gga_sats) == NULL) {
			break;
		}
		in.sats = strtoul(p + 1, NULL, 10);

		/* horizontal dilution of precision read */
		if (nmea_nField(&p, str, field_gga_hdop) == NULL) {
			break;
		}
		in.hdop = strtod(p + 1, NULL);

		/* Antenna altitude above mean-sea-level read */
		if (nmea_nField(&p, str, field_gga_h_asl) == NULL) {
			break;
		}
		in.h_asl = strtod(p + 1, NULL);

		/* Geoidal separation read */
		if (nmea_nField(&p, str, field_gga_h_wgs) == NULL) {
			break;
		}
		in.h_wgs = strtod(p + 1, NULL);

		out->msg.gga = in;
		out->type = nmea_gga;
		return nmea_gga;

	} while (0);

	return nmea_broken;
}


static int nmea_parsermc(char *str, nmea_t *out)
{
	char *p, *sign;
	nmea_rmc_t in;

	do {
		/* latitude read */
		if (nmea_nField(&p, str, field_rmc_lat) == NULL || nmea_nField(&sign, str, field_rmc_lat + 1) == NULL) {
			break;
		}
		in.lat = strtod(p + 1, NULL);
		in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
		in.lat = (*(sign + 1) == 'S') ? -in.lat : in.lat;

		/* longitude read */
		if (nmea_nField(&p, str, field_rmc_lon) == NULL || nmea_nField(&sign, str, field_rmc_lon + 1) == NULL) {
			break;
		}
		in.lon = strtod(p + 1, NULL);
		in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
		in.lon = (*(sign + 1) == 'S') ? -in.lon : in.lon;

		/* speed in knots read */
		if (nmea_nField(&p, str, field_rmc_speedknots) == NULL) {
			break;
		}
		in.speed = strtod(p + 1, NULL);

		/* course read */
		if (nmea_nField(&p, str, field_rmc_course) == NULL) {
			break;
		}
		in.course = strtod(p + 1, NULL);

		/* magnetic variation read */
		if (nmea_nField(&p, str, field_rmc_magvar) == NULL) {
			break;
		}
		in.magvar = strtod(p + 1, NULL);

		out->msg.rmc = in;
		out->type = nmea_rmc;
		return nmea_rmc;

	} while (0);

	return nmea_broken;
}


int nmea_interpreter(char *str, nmea_t *out)
{
	int ret = EOK;

	if (strncmp(str, "$GPGSA", 6) == 0) {
		ret = nmea_parsegsa(str, out);
	}
	else if (strncmp(str, "$GPVTG", 6) == 0) {
		ret = nmea_parsevtg(str, out);
	}
	else if (strncmp(str, "$GPGGA", 6) == 0) {
		ret = nmea_parsegga(str, out);
	}
	else if (strncmp(str, "$GPRMC", 6) == 0) {
		ret = nmea_parsermc(str, out);
	}
	else {
		ret = nmea_unknown;
	}

	return ret;
}
