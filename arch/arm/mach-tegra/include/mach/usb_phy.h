/*
 * arch/arm/mach-tegra/include/mach/usb_phy.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
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

#ifndef __MACH_USB_PHY_H
#define __MACH_USB_PHY_H

#include <linux/platform_device.h>
#include <linux/clk.h>

#define USB_PHY_MAX_CONTEXT_REGS 10

struct tegra_utmip_config {
	u8 hssync_start_delay;
	u8 elastic_limit;
	u8 idle_wait_delay;
	u8 term_range_adj;
	u8 xcvr_setup;
	u8 xcvr_lsfslew;
	u8 xcvr_lsrslew;
};

enum tegra_ulpi_inf_type {
	TEGRA_USB_LINK_ULPI = 0,
	TEGRA_USB_NULL_ULPI,
	TEGRA_USB_UHSIC,
};

struct tegra_ulpi_trimmer {
	u8 shadow_clk_delay;	/* 0 ~ 31 */
	u8 clock_out_delay;	/* 0 ~ 31 */
	u8 data_trimmer;	/* 0 ~ 7 */
	u8 stpdirnxt_trimmer;	/* 0 ~ 7 */
};

struct tegra_ulpi_config {
	enum tegra_ulpi_inf_type inf_type;
	int reset_gpio;
	const char *clk;
	const struct tegra_ulpi_trimmer *trimmer;
	int (*preinit)(void);
	int (*postinit)(void);
};

struct tegra_uhsic_config {
	u8 sync_start_delay;
	u8 idle_wait_delay;
	u8 term_range_adj;
	u8 elastic_underrun_limit;
	u8 elastic_overrun_limit;
};

enum tegra_usb_phy_port_speed {
	TEGRA_USB_PHY_PORT_SPEED_FULL = 0,
	TEGRA_USB_PHY_PORT_SPEED_LOW,
	TEGRA_USB_PHY_PORT_HIGH,
};

enum tegra_usb_phy_mode {
	TEGRA_USB_PHY_MODE_DEVICE,
	TEGRA_USB_PHY_MODE_HOST,
};

struct tegra_usb_phy {
	int instance;
	int freq_sel;
	void __iomem *regs;
	void __iomem *pad_regs;
	struct clk *clk;
	struct clk *pll_u;
	struct clk *pad_clk;
	enum tegra_usb_phy_mode mode;
	void *config;
};

struct tegra_usb_phy *tegra_usb_phy_open(int instance, void __iomem *regs,
			void *config, enum tegra_usb_phy_mode phy_mode);

int tegra_usb_phy_power_on(struct tegra_usb_phy *phy);

int tegra_usb_phy_clk_disable(struct tegra_usb_phy *phy);

int tegra_usb_phy_clk_enable(struct tegra_usb_phy *phy);

int tegra_usb_phy_power_off(struct tegra_usb_phy *phy);

int tegra_usb_phy_preresume(struct tegra_usb_phy *phy);

int tegra_usb_phy_postresume(struct tegra_usb_phy *phy);

int tegra_ehci_phy_restore_start(struct tegra_usb_phy *phy,
				 enum tegra_usb_phy_port_speed port_speed);

int tegra_ehci_phy_restore_end(struct tegra_usb_phy *phy);

int tegra_usb_phy_close(struct tegra_usb_phy *phy);

int tegra_usb_phy_bus_connect(struct tegra_usb_phy *phy);

int tegra_usb_phy_bus_reset(struct tegra_usb_phy *phy);

int tegra_usb_phy_bus_idle(struct tegra_usb_phy *phy);

bool tegra_usb_phy_is_device_connected(struct tegra_usb_phy *phy);

#endif /*__MACH_USB_PHY_H */
