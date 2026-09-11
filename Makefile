CC ?= gcc
CXX ?= g++
PKG_CONFIG ?= pkg-config
BUILD_DIR := build
CPPFLAGS := -D_GNU_SOURCE -Iinclude -Iapp/controller
CFLAGS := -std=c11 -O2 -Wall -Wextra -Wpedantic
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic

CONTROLLER_SOURCES := \
	app/controller/main.c \
	app/controller/controller.c \
	app/controller/request_queue.c \
	app/controller/motor_backend.c \
	app/controller/motor_service.c \
	app/controller/tcp_server.c \
	app/controller/camera_ipc.c

.PHONY: all app tools vision driver clean

all: app tools

app: $(BUILD_DIR)/wall_robot_controller

tools: $(BUILD_DIR)/command_client $(BUILD_DIR)/motor_cli

vision: $(BUILD_DIR)/camera_app

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/wall_robot_controller: $(CONTROLLER_SOURCES) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -pthread -o $@

$(BUILD_DIR)/command_client: tools/command_client.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o $@

$(BUILD_DIR)/motor_cli: tools/motor_cli.c include/dual_stepper_uapi.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o $@

$(BUILD_DIR)/camera_app: app/vision/camera_app.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< $$( $(PKG_CONFIG) --cflags --libs opencv4 ) -o $@

driver:
	$(MAKE) -C /lib/modules/$$(uname -r)/build M=$(CURDIR)/driver modules

clean:
	$(RM) $(BUILD_DIR)/wall_robot_controller \
	      $(BUILD_DIR)/command_client \
	      $(BUILD_DIR)/motor_cli \
	      $(BUILD_DIR)/camera_app
