/**
 * @file
 * @brief Minimal RGB LED hardware smoke test for dust-hpm6750.
 *
 * This image intentionally exercises only the Zephyr kernel and the three GPIO
 * pins described by the board's led0, led1, and led2 aliases.
 */

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LED_R_NODE DT_ALIAS(led0)
#define LED_G_NODE DT_ALIAS(led1)
#define LED_B_NODE DT_ALIAS(led2)

#if !DT_NODE_HAS_STATUS(LED_R_NODE, okay) || \
	!DT_NODE_HAS_STATUS(LED_G_NODE, okay) || \
	!DT_NODE_HAS_STATUS(LED_B_NODE, okay)
#error "dust-hpm6750 must define enabled led0, led1, and led2 aliases"
#endif

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(LED_G_NODE, gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(LED_B_NODE, gpios);

static int configure_led(const struct gpio_dt_spec *led)
{
	if (!gpio_is_ready_dt(led)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
}

static int configure_rgb(void)
{
	int ret;

	ret = configure_led(&led_r);
	if (ret != 0) {
		return ret;
	}

	ret = configure_led(&led_g);
	if (ret != 0) {
		return ret;
	}

	return configure_led(&led_b);
}

static void set_rgb(bool red, bool green, bool blue)
{
	(void)gpio_pin_set_dt(&led_r, red);
	(void)gpio_pin_set_dt(&led_g, green);
	(void)gpio_pin_set_dt(&led_b, blue);
}

static void report_phase(const char *phase)
{
	if (IS_ENABLED(CONFIG_PRINTK)) {
		printk("RGB phase: %s\n", phase);
	}
}

int main(void)
{
	if (configure_rgb() != 0) {
		/* No console is enabled: stay stopped so a setup failure cannot be
		 * mistaken for a valid or intermittently resetting LED sequence.
		 */
		for (;;) {
			k_sleep(K_FOREVER);
		}
	}

	for (;;) {
		set_rgb(false, false, false);
		report_phase("off-short");
		k_sleep(K_MSEC(250));

		set_rgb(true, false, false);
		report_phase("red");
		k_sleep(K_MSEC(750));

		set_rgb(false, true, false);
		report_phase("green");
		k_sleep(K_MSEC(750));

		set_rgb(false, false, true);
		report_phase("blue");
		k_sleep(K_MSEC(750));

		set_rgb(true, true, true);
		report_phase("white");
		k_sleep(K_MSEC(750));

		set_rgb(false, false, false);
		report_phase("off-long");
		k_sleep(K_MSEC(750));
	}

	return 0;
}
