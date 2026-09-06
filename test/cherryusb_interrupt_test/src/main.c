#include <zephyr/sys/printk.h>
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file test/cherryusb_interrupt_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <stdint.h>
#include <stdio.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

extern void int_test_init(uint8_t busid, uint32_t reg_base);

int main(void)
{
	uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

	printk("cherryusb interrupt test booted.\n");

	int_test_init(0, usb_base);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
