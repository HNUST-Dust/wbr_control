/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

namespace {

constexpr int kBannerRetryMs = 200;
constexpr int kBannerMaxRetry = 20;

K_WORK_DELAYABLE_DEFINE(g_shell_banner_work, nullptr);
int g_shell_banner_retry = 0;
bool g_shell_banner_printed = false;

void ShellBannerWorkHandler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (g_shell_banner_printed) {
		return;
	}

	const struct shell *sh = shell_backend_uart_get_ptr();
	if ((sh == nullptr) || !shell_ready(sh)) {
		if (g_shell_banner_retry < kBannerMaxRetry) {
			++g_shell_banner_retry;
			(void)k_work_reschedule(&g_shell_banner_work, K_MSEC(kBannerRetryMs));
		}
		return;
	}

	shell_print(sh, "");
	shell_print(sh, " _____   _   _   _____   _____ ");
	shell_print(sh, "|  __ \\ | | | | / ____| |_   _|");
	shell_print(sh, "| |  | || | | || (___     | |  ");
	shell_print(sh, "| |  | || | | | \\___ \\    | |  ");
	shell_print(sh, "| |__| || |_| | ____) |   | |  ");
	shell_print(sh, "|_____/  \\___/ |_____/    |_|  ");
	shell_print(sh, "Type 'help' to list commands.");
	shell_print(sh, "Type 'chassis pid status|get|set|save|load|dump'.");
	shell_print(sh, "-------------- WBR_CONTROL SHELL ------------------");
	g_shell_banner_printed = true;
}

int ShellBannerInit(void)
{
	k_work_init_delayable(&g_shell_banner_work, ShellBannerWorkHandler);
	(void)k_work_reschedule(&g_shell_banner_work, K_MSEC(300));
	return 0;
}

SYS_INIT(ShellBannerInit, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

}  // namespace
