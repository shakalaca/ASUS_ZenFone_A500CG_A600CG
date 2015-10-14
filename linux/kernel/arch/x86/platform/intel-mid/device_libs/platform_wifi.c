/*
 * platform_bcm43xx.c: bcm43xx platform data initilization file
 *
 * (C) Copyright 2011 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <asm/intel-mid.h>
#include <linux/wlan_plat.h>
#include <linux/interrupt.h>
#include <linux/mmc/sdhci.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "pci/platform_sdhci_pci.h"
#include "platform_wifi.h"
/* Delay copied from broadcom reference design */
#define DELAY_ONOFF 250

static int gpio_enable;
static void (*g_virtual_cd)(void *dev_id, int card_present);
void *g_host;

void bcmdhd_register_embedded_control(void *dev_id,
			void (*virtual_cd)(void *dev_id, int card_present))
{
	g_virtual_cd = virtual_cd;
	g_host = dev_id;
}

static int bcmdhd_set_power(int on)
{
	gpio_set_value(gpio_enable, on);

	/* Delay advice by BRCM */
	msleep(DELAY_ONOFF);
	return 0;
}

static int bcmdhd_set_card_detect(int detect)
{
        if (!g_virtual_cd)
		return -1;

	if (g_host)
		g_virtual_cd(g_host, detect);
	return 0;
}

static struct wifi_platform_data bcmdhd_data = {
	.set_power = bcmdhd_set_power,
	.set_carddetect = bcmdhd_set_card_detect,
};

static struct resource wifi_res[] = {
	{
	.name = "bcmdhd_wlan_irq",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ | IRQF_TRIGGER_RISING ,
	},
	{
	.name = "bcmdhd_wlan_en",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ ,
	}

};

static struct platform_device wifi_device = {
	.name = "bcmdhd_wlan",
	.dev = {
		.platform_data = &bcmdhd_data,
		},
	.num_resources = ARRAY_SIZE(wifi_res),
	.resource = wifi_res,
};

void __init wifi_platform_data_init_sfi(void)
{
	int err;
	int wifi_irq_gpio = -1;

	pr_err("wifi_platform_data_init_sfi\n");

	/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(WIFI_SFI_GPIO_IRQ_NAME);
	if (wifi_irq_gpio < 0) {
		pr_err("%s: Unable to find" WIFI_SFI_GPIO_IRQ_NAME
		       "WLAN-interrupt GPIO in the SFI table\n",
		       __func__);
		return;
	}

	wifi_res[0].start = wifi_irq_gpio;
	wifi_res[0].end = wifi_res[0].start;
	gpio_enable = get_gpio_by_name(WIFI_SFI_GPIO_ENABLE_NAME);
	if (gpio_enable < 0) {
		pr_err("%s: Unable to find WLAN_EN GPIO in the SFI table\n",
		       __func__);
		return;
	}

	wifi_res[1].start = gpio_enable;
	wifi_res[1].end = wifi_res[1].start;

	err = platform_device_register(&wifi_device);
	if (err < 0)
		pr_err("platform_device_register failed for wifi_device\n");
}


/* Called from board.c */
void __init *wifi_platform_data(void *info)
{
	struct sd_board_info *sd_info = info;

	unsigned int sdhci_quirk = SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8
                | SDHCI_QUIRK2_ENABLE_MMC_PM_IGNORE_PM_NOTIFY
                | SDHCI_QUIRK2_ADVERTISE_3V0_FORCE_1V8
                | SDHCI_QUIRK2_NON_STD_CIS;

	pr_err("Using generic wifi platform data\n");

	/* Set vendor specific SDIO quirks */
	sdhci_pdata_set_quirks(sdhci_quirk);
	sdhci_pdata_set_embedded_control(&bcmdhd_register_embedded_control);

#ifndef CONFIG_ACPI
	/* We are SFI here, register platform device */
	wifi_platform_data_init_sfi();
#endif

	return &wifi_device;
}
