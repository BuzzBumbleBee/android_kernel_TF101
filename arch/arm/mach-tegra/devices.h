/*
 * arch/arm/mach-tegra/devices.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Erik Gilling <ccross@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_DEVICES_H
#define __MACH_TEGRA_DEVICES_H

#include <linux/platform_device.h>

extern struct platform_device tegra_sdhci_device1;
extern struct platform_device tegra_sdhci_device2;
extern struct platform_device tegra_sdhci_device3;
extern struct platform_device tegra_sdhci_device4;
extern struct platform_device tegra_i2c_device1;
extern struct platform_device tegra_i2c_device2;
extern struct platform_device tegra_i2c_device3;
extern struct platform_device tegra_i2c_device4;
extern struct platform_device tegra_spi_device1;
extern struct platform_device tegra_spi_device2;
extern struct platform_device tegra_spi_device3;
extern struct platform_device tegra_spi_device4;
extern struct platform_device tegra_w1_device;
extern struct platform_device tegra_udc_device;
extern struct platform_device tegra_ehci1_device;
extern struct platform_device tegra_ehci2_device;
extern struct platform_device tegra_ehci3_device;
extern struct platform_device tegra_i2s_device1;
extern struct platform_device tegra_i2s_device2;
extern struct platform_device tegra_gart_device;
extern struct platform_device pmu_device;
extern struct platform_device tegra_wdt_device;
extern struct platform_device tegra_pwfm0_device;
extern struct platform_device tegra_pwfm1_device;
extern struct platform_device tegra_pwfm2_device;
extern struct platform_device tegra_pwfm3_device;
extern struct platform_device tegra_otg_device;
extern struct platform_device tegra_uarta_device;
extern struct platform_device tegra_uartb_device;
extern struct platform_device tegra_uartc_device;
extern struct platform_device tegra_uartd_device;
extern struct platform_device tegra_uarte_device;
extern struct platform_device tegra_spdif_device;
extern struct platform_device tegra_grhost_device;
extern struct platform_device tegra_spdif_device;
extern struct platform_device tegra_avp_device;
extern struct platform_device tegra_aes_device;
extern struct platform_device tegra_das_device;

#endif
