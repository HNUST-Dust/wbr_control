/**
 * @file test/pwm_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_NODE DT_NODELABEL(pwm0)

#if !DT_NODE_HAS_STATUS(PWM_NODE, okay)
#error "pwm0 is not enabled in devicetree"
#endif

#define PWM_FREQUENCY_HZ DT_PROP(PWM_NODE, period_init)

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_NODE);

static int set_pwm_channel(uint32_t duty_percent)
{
	uint64_t cycles_per_sec;
	uint32_t period_cycles;
	uint32_t pulse_cycles;
	int rc;

	if (!device_is_ready(pwm_dev)) {
		printk("PWM device is not ready\n");
		return -ENODEV;
	}

	rc = pwm_get_cycles_per_sec(pwm_dev, 6, &cycles_per_sec);
	if (rc != 0) {
		printk("Failed to get PWM clock rate (err %d)\n", rc);
		return rc;
	}

	period_cycles = (uint32_t)(cycles_per_sec / PWM_FREQUENCY_HZ);
	if (period_cycles == 0U) {
		period_cycles = 1U;
	}

	pulse_cycles = (period_cycles * duty_percent) / 100U;

	rc = pwm_set_cycles(pwm_dev, 6, period_cycles, pulse_cycles, PWM_POLARITY_NORMAL);
	if (rc != 0) {
		printk("Failed to set PWM0 channel 6 (err %d)\n", rc);
		return rc;
	}

	printk("PWM0: period=%u Hz, ch6=%u%%\n", PWM_FREQUENCY_HZ, duty_percent);
	return 0;
}

int main(void)
{
	static const uint32_t duty_points[] = {
		10,
		25,
		50,
		75,
		90,
	};
	const size_t point_count = ARRAY_SIZE(duty_points);
	size_t index = 0;
	uint64_t cycles_per_sec;
	uint32_t period_cycles;
	int rc;

	printk("pwm_test booted\n");

	/* Workaround for hpmicro PWM v1 driver bug:
	 * The driver only calls pwm_setup_waveform() when period changes.
	 * We deliberately pass a different period first to trigger
	 * the init branch and start the counter.
	 */
	rc = pwm_get_cycles_per_sec(pwm_dev, 6, &cycles_per_sec);
	if (rc == 0) {
		period_cycles = (uint32_t)(cycles_per_sec / PWM_FREQUENCY_HZ);
		if (period_cycles == 0U) {
			period_cycles = 1U;
		}
		rc = pwm_set_cycles(pwm_dev, 6, period_cycles + 1,
				    (period_cycles + 1) / 2, PWM_POLARITY_NORMAL);
		if (rc == 0) {
			k_sleep(K_MSEC(10));
		}
	}

	while (1) {
		(void)set_pwm_channel(duty_points[index]);
		index = (index + 1U) % point_count;
		k_sleep(K_SECONDS(2));
	}

	return 0;
}