// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>

int main()
{
	while (true) {
		k_sleep(K_FOREVER);
	}
	return 0;
}
