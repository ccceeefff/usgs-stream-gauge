#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
CWD = $(shell pwd)
IDF_PATH = $(CWD)/../esp-idf

PROJECT_NAME := usgs-stream-gauge

# go up to main app and reference components dir from there
EXTRA_COMPONENT_DIRS = $(PROJECT_PATH)/../lib
EXTRA_COMPONENT_DIRS += $(PROJECT_PATH)/../boards/envisense/v1.1

include $(IDF_PATH)/make/project.mk
