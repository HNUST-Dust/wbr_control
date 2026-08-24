/**
 * @file test/printk_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

struct uart_target {
	const struct device *dev;
	const char *name;
};

static void send_line(const struct uart_target *target, const char *line)
{
	if (!device_is_ready(target->dev)) {
		printk("%s ready=0\n", target->name);
		return;
	}

	printk("%s ready=1\n", target->name);

	for (const char *ch = line; *ch != '\0'; ++ch) {
		uart_poll_out(target->dev, *ch);
	}

	uart_poll_out(target->dev, '\r');
	uart_poll_out(target->dev, '\n');
}

int main(void)
{
	uint32_t counter = 0U;
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	static const struct uart_target targets[] = {
		{ DEVICE_DT_GET(DT_NODELABEL(uart2)), "uart2" },
		{ DEVICE_DT_GET(DT_NODELABEL(uart3)), "uart3" },
		{ DEVICE_DT_GET(DT_NODELABEL(uart10)), "uart10" },
		{ DEVICE_DT_GET(DT_NODELABEL(uart11)), "uart11" },
		{ DEVICE_DT_GET(DT_NODELABEL(uart12)), "uart12" },
		{ DEVICE_DT_GET(DT_NODELABEL(uart15)), "uart15" },
	};
	const int console_ready = device_is_ready(console);

	printk("printk_test booted\n");
	printk("ready: console=%d\n", console_ready);

	for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); ++i) {
		printk("ready: %s=%d\n", targets[i].name, device_is_ready(targets[i].dev));
	}

	while (1) {
		char line[64];

		snprintk(line, sizeof(line), "printk_test heartbeat: %u", counter);
		printk("printk_test heartbeat: %u\n", counter);

		for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); ++i) {
			send_line(&targets[i], line);
		}

		counter++;
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
