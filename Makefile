RACK_DIR ?= ../..


FLAGS += -I./dsp

SOURCES += $(wildcard src/*.cpp)
DISTRIBUTABLES += $(wildcard LICENSE*) res



include $(RACK_DIR)/plugin.mk
