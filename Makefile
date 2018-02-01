#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := pca9685_pwm_test

EXTRA_CFLAGS += --save-temps

include $(IDF_PATH)/make/project.mk

