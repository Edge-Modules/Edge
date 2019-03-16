RACK_DIR ?= ../..
SLUG = Edge
VERSION = 0.6.4

FLAGS += -I./dsp

SOURCES += $(wildcard src/*.cpp)
DISTRIBUTABLES += $(wildcard LICENSE*) res



include $(RACK_DIR)/plugin.mk
