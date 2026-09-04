/**
 * @file test/rgb_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

/* Aliases defined in the board DTS: led0 -> red, led1 -> green, led2 -> blue */
#define LED_R_NODE DT_ALIAS(led0)
#define LED_G_NODE DT_ALIAS(led1)
#define LED_B_NODE DT_ALIAS(led2)

#if !DT_NODE_EXISTS(LED_R_NODE) || !DT_NODE_EXISTS(LED_G_NODE) || !DT_NODE_EXISTS(LED_B_NODE)
#error "One or more LED aliases (led0/led1/led2) are not defined in devicetree"
#endif

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(LED_G_NODE, gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(LED_B_NODE, gpios);

static void init_led(const struct gpio_dt_spec *led, const char *name)
{
    if (!device_is_ready(led->port)) {
        printk("%s GPIO device not ready\n", name);
        return;
    }

    int rc = gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
    if (rc != 0) {
        printk("Failed to configure %s (err %d)\n", name, rc);
    }
}

static void set_led(const struct gpio_dt_spec *led, bool on)
{
    if (!device_is_ready(led->port)) {
        return;
    }
    gpio_pin_set(led->port, led->pin, (int)on);
}

int main(void)
{
    printk("rgb_test booted\n");

    init_led(&led_r, "led_r");
    init_led(&led_g, "led_g");
    init_led(&led_b, "led_b");

    while (1) {
        /* Red */
        set_led(&led_r, true);
        set_led(&led_g, false);
        set_led(&led_b, false);
        printk("LED: RED\n");
        k_sleep(K_MSEC(500));

        /* Green */
        set_led(&led_r, false);
        set_led(&led_g, true);
        set_led(&led_b, false);
        printk("LED: GREEN\n");
        k_sleep(K_MSEC(500));

        /* Blue */
        set_led(&led_r, false);
        set_led(&led_g, false);
        set_led(&led_b, true);
        printk("LED: BLUE\n");
        k_sleep(K_MSEC(500));

        /* White (all on) */
        set_led(&led_r, true);
        set_led(&led_g, true);
        set_led(&led_b, true);
        printk("LED: WHITE\n");
        k_sleep(K_MSEC(500));
    }

    return 0;
}
