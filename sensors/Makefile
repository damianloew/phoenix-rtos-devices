#
# Makefile for Phoenix-RTOS sensor manager
#
# Copyright 2022 Phoenix Systems
#

# Sensors Server API Library
NAME := libsensors
include $(static-lib.mk)


# Sensors Server
NAME := sensors
LOCAL_HEADERS_DIR := nothing
LOCAL_SRCS := sensors.c
LOCAL_SRCS += imu/lsm9dsxx.c
LOCAL_SRCS += baro/lps25xx.c
DEP_LIBS := libsensors libspi-msg
include $(binary.mk)
