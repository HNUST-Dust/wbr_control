# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cherryusb_interrupt_test)

target_sources(app PRIVATE
	src/main.c
	src/int_test.c
)

set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
