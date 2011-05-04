/*
 * arch/arm/mach-tegra/usb_phy.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 - 2011 NVIDIA Corporation
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *	Benoit Goby <benoit@android.com>
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

#include <linux/resource.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/usb_phy.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include "gpio-names.h"

#define USB_USBCMD		0x140
#define   USB_USBCMD_RS		(1 << 0)

#define USB_USBSTS		0x144
#define   USB_USBSTS_PCI	(1 << 2)
#define   USB_USBSTS_HCH	(1 << 12)

#define USB_TXFILLTUNING        0x164

#define ULPI_VIEWPORT		0x170
#define   ULPI_WAKEUP		(1 << 31)
#define   ULPI_RUN		(1 << 30)
#define   ULPI_RD_RW_WRITE	(1 << 29)
#define   ULPI_RD_RW_READ	(0 << 29)
#define   ULPI_PORT(x)		(((x) & 0x7) << 24)
#define   ULPI_ADDR(x)		(((x) & 0xff) << 16)
#define   ULPI_DATA_RD(x)	(((x) & 0xff) << 8)
#define   ULPI_DATA_WR(x)	(((x) & 0xff) << 0)

#define USB_PORTSC1		0x184
#define   USB_PORTSC1_PTS(x)	(((x) & 0x3) << 30)
#define   USB_PORTSC1_PSPD(x)	(((x) & 0x3) << 26)
#define   USB_PORTSC1_PHCD	(1 << 23)
#define   USB_PORTSC1_WKOC	(1 << 22)
#define   USB_PORTSC1_WKDS	(1 << 21)
#define   USB_PORTSC1_WKCN	(1 << 20)
#define   USB_PORTSC1_PTC(x)	(((x) & 0xf) << 16)
#define   USB_PORTSC1_PP	(1 << 12)
#define   USB_PORTSC1_LS(x)	(((x) & 0x3) << 10)
#define   USB_PORTSC1_SUSP	(1 << 7)
#define   USB_PORTSC1_PE	(1 << 2)
#define   USB_PORTSC1_CCS	(1 << 0)

#define USB_SUSP_CTRL		0x400
#define   USB_WAKE_ON_CNNT_EN_DEV	(1 << 3)
#define   USB_WAKE_ON_DISCON_EN_DEV	(1 << 4)
#define   USB_SUSP_CLR		(1 << 5)
#define   USB_PHY_CLK_VALID	(1 << 7)
#define   UTMIP_RESET		(1 << 11)
#define   UHSIC_RESET		(1 << 11)
#define   UTMIP_PHY_ENABLE	(1 << 12)
#define   UHSIC_PHY_ENABLE	(1 << 12)
#define   ULPI_PHY_ENABLE	(1 << 13)
#define   USB_SUSP_SET		(1 << 14)
#define   USB_WAKEUP_DEBOUNCE_COUNT(x)	(((x) & 0x7) << 16)

#define USB1_LEGACY_CTRL	0x410
#define   USB1_NO_LEGACY_MODE			(1 << 0)
#define   USB1_VBUS_SENSE_CTL_MASK		(3 << 1)
#define   USB1_VBUS_SENSE_CTL_VBUS_WAKEUP	(0 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD_OR_VBUS_WAKEUP \
						(1 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD	(2 << 1)
#define   USB1_VBUS_SENSE_CTL_A_SESS_VLD	(3 << 1)

#define ULPIS2S_CTRL		0x418
#define   ULPIS2S_ENA			(1 << 0)
#define   ULPIS2S_SUPPORT_DISCONNECT	(1 << 2)
#define   ULPIS2S_PLLU_MASTER_BLASTER60	(1 << 3)
#define   ULPIS2S_SPARE(x)		(((x) & 0xF) << 8)
#define   ULPIS2S_FORCE_ULPI_CLK_OUT	(1 << 12)
#define   ULPIS2S_DISCON_DONT_CHECK_SE0	(1 << 13)
#define   ULPIS2S_SUPPORT_HS_KEEP_ALIVE (1 << 14)
#define   ULPIS2S_DISABLE_STP_PU	(1 << 15)

#define ULPI_TIMING_CTRL_0	0x424
#define   ULPI_CLOCK_OUT_DELAY(x)	((x) & 0x1F)
#define   ULPI_OUTPUT_PINMUX_BYP	(1 << 10)
#define   ULPI_CLKOUT_PINMUX_BYP	(1 << 11)
#define   ULPI_SHADOW_CLK_LOOPBACK_EN	(1 << 12)
#define   ULPI_SHADOW_CLK_SEL		(1 << 13)
#define   ULPI_CORE_CLK_SEL		(1 << 14)
#define   ULPI_SHADOW_CLK_DELAY(x)	(((x) & 0x1F) << 16)
#define   ULPI_LBK_PAD_EN		(1 << 26)
#define   ULPI_LBK_PAD_E_INPUT_OR	(1 << 27)
#define   ULPI_CLK_OUT_ENA		(1 << 28)
#define   ULPI_CLK_PADOUT_ENA		(1 << 29)

#define ULPI_TIMING_CTRL_1	0x428
#define   ULPI_DATA_TRIMMER_LOAD	(1 << 0)
#define   ULPI_DATA_TRIMMER_SEL(x)	(((x) & 0x7) << 1)
#define   ULPI_STPDIRNXT_TRIMMER_LOAD	(1 << 16)
#define   ULPI_STPDIRNXT_TRIMMER_SEL(x)	(((x) & 0x7) << 17)
#define   ULPI_DIR_TRIMMER_LOAD		(1 << 24)
#define   ULPI_DIR_TRIMMER_SEL(x)	(((x) & 0x7) << 25)

#define UTMIP_PLL_CFG1		0x804
#define   UTMIP_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UTMIP_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 27)

#define UTMIP_XCVR_CFG0		0x808
#define   UTMIP_XCVR_SETUP(x)			(((x) & 0xf) << 0)
#define   UTMIP_XCVR_LSRSLEW(x)			(((x) & 0x3) << 8)
#define   UTMIP_XCVR_LSFSLEW(x)			(((x) & 0x3) << 10)
#define   UTMIP_FORCE_PD_POWERDOWN		(1 << 14)
#define   UTMIP_FORCE_PD2_POWERDOWN		(1 << 16)
#define   UTMIP_FORCE_PDZI_POWERDOWN		(1 << 18)
#define   UTMIP_XCVR_HSSLEW_MSB(x)		(((x) & 0x7f) << 25)

#define UTMIP_BIAS_CFG0		0x80c
#define   UTMIP_OTGPD			(1 << 11)
#define   UTMIP_BIASPD			(1 << 10)

#define UTMIP_HSRX_CFG0		0x810
#define   UTMIP_ELASTIC_LIMIT(x)	(((x) & 0x1f) << 10)
#define   UTMIP_IDLE_WAIT(x)		(((x) & 0x1f) << 15)

#define UTMIP_HSRX_CFG1		0x814
#define   UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)

#define UTMIP_TX_CFG0		0x820
#define   UTMIP_FS_PREABMLE_J		(1 << 19)
#define   UTMIP_HS_DISCON_DISABLE	(1 << 8)

#define UTMIP_MISC_CFG0		0x824
#define   UTMIP_DPDM_OBSERVE		(1 << 26)
#define   UTMIP_DPDM_OBSERVE_SEL(x)	(((x) & 0xf) << 27)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_J	UTMIP_DPDM_OBSERVE_SEL(0xf)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_K	UTMIP_DPDM_OBSERVE_SEL(0xe)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE1 UTMIP_DPDM_OBSERVE_SEL(0xd)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE0 UTMIP_DPDM_OBSERVE_SEL(0xc)
#define   UTMIP_SUSPEND_EXIT_ON_EDGE	(1 << 22)

#define UTMIP_MISC_CFG1		0x828
#define   UTMIP_PLL_ACTIVE_DLY_COUNT(x)	(((x) & 0x1f) << 18)
#define   UTMIP_PLLU_STABLE_COUNT(x)	(((x) & 0xfff) << 6)

#define UTMIP_DEBOUNCE_CFG0	0x82c
#define   UTMIP_BIAS_DEBOUNCE_A(x)	(((x) & 0xffff) << 0)

#define UTMIP_BAT_CHRG_CFG0	0x830
#define   UTMIP_PD_CHRG			(1 << 0)

#define UTMIP_SPARE_CFG0	0x834
#define   FUSE_SETUP_SEL		(1 << 3)

#define UTMIP_XCVR_CFG1		0x838
#define   UTMIP_FORCE_PDDISC_POWERDOWN	(1 << 0)
#define   UTMIP_FORCE_PDCHRP_POWERDOWN	(1 << 2)
#define   UTMIP_FORCE_PDDR_POWERDOWN	(1 << 4)
#define   UTMIP_XCVR_TERM_RANGE_ADJ(x)	(((x) & 0xf) << 18)

#define UTMIP_BIAS_CFG1		0x83c
#define   UTMIP_BIAS_PDTRK_COUNT(x)	(((x) & 0x1f) << 3)

//add by yi-hsin for check ulpi phy fail
static int ulpi_phy_fail = 0;
#define UHSIC_PLL_CFG0				0x800

#define UHSIC_PLL_CFG1				0x804
#define   UHSIC_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UHSIC_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 14)

#define UHSIC_HSRX_CFG0				0x808
#define   UHSIC_ELASTIC_UNDERRUN_LIMIT(x)	(((x) & 0x1f) << 2)
#define   UHSIC_ELASTIC_OVERRUN_LIMIT(x)	(((x) & 0x1f) << 8)
#define   UHSIC_IDLE_WAIT(x)			(((x) & 0x1f) << 13)

#define UHSIC_HSRX_CFG1				0x80c
#define   UHSIC_HS_SYNC_START_DLY(x)		(((x) & 0x1f) << 1)

#define UHSIC_TX_CFG0				0x810
#define   UHSIC_HS_POSTAMBLE_OUTPUT_ENABLE	(1 << 6)

#define UHSIC_MISC_CFG0				0x814
#define   UHSIC_SUSPEND_EXIT_ON_EDGE		(1 << 7)
#define   UHSIC_DETECT_SHORT_CONNECT		(1 << 8)
#define   UHSIC_FORCE_XCVR_MODE			(1 << 15)

#define UHSIC_MISC_CFG1				0X818
#define   UHSIC_PLLU_STABLE_COUNT(x)		(((x) & 0xfff) << 2)

#define UHSIC_PADS_CFG0				0x81c
#define   UHSIC_TX_RTUNEN			0xf000
#define   UHSIC_TX_RTUNE(x)			(((x) & 0xf) << 12)

#define UHSIC_PADS_CFG1				0x820
#define   UHSIC_PD_BG				(1 << 2)
#define   UHSIC_PD_TX				(1 << 3)
#define   UHSIC_PD_TRK				(1 << 4)
#define   UHSIC_PD_RX				(1 << 5)
#define   UHSIC_PD_ZI				(1 << 6)
#define   UHSIC_RX_SEL				(1 << 7)
#define   UHSIC_RPD_DATA			(1 << 9)
#define   UHSIC_RPD_STROBE			(1 << 10)
#define   UHSIC_RPU_DATA			(1 << 11)
#define   UHSIC_RPU_STROBE			(1 << 12)

#define UHSIC_CMD_CFG0				0x824
#define   UHSIC_PRETEND_CONNECT_DETECT		(1 << 5)

#define UHSIC_STAT_CFG0				0x828
#define   UHSIC_CONNECT_DETECT			(1 << 0)

#define UHSIC_SPARE_CFG0			0x82c

static DEFINE_SPINLOCK(utmip_pad_lock);
static int utmip_pad_count;

static const int udc_freq_table[] = {
	12000000,
	13000000,
	19200000,
	26000000,
};

static const u8 udc_delay_table[][4] = {
	/* ENABLE_DLY, STABLE_CNT, ACTIVE_DLY, XTAL_FREQ_CNT */
	{0x02,         0x2F,       0x04,       0x76}, /* 12 MHz */
	{0x02,         0x33,       0x05,       0x7F}, /* 13 MHz */
	{0x03,         0x4B,       0x06,       0xBB}, /* 19.2 MHz */
	{0x04,         0x66,       0x09,       0xFE}, /* 26 Mhz */
};

static const u16 uhsic_delay_table[][4] = {
	/* ENABLE_DLY, STABLE_CNT, ACTIVE_DLY, XTAL_FREQ_CNT */
	{0x02,         0x2F,       0x0,       0x1CA}, /* 12 MHz */
	{0x02,         0x33,       0x0,       0x1F0}, /* 13 MHz */
	{0x03,         0x4B,       0x0,       0x2DD}, /* 19.2 MHz */
	{0x04,         0x66,       0x0,       0x3E0}, /* 26 Mhz */
};

static const u16 udc_debounce_table[] = {
	0x7530, /* 12 MHz */
	0x7EF4, /* 13 MHz */
	0xBB80, /* 19.2 MHz */
	0xFDE8, /* 26 MHz */
};

static struct tegra_utmip_config utmip_default[] = {
	[0] = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 9,
		.xcvr_lsfslew = 1,
		.xcvr_lsrslew = 1,
	},
	[2] = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 9,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static struct tegra_uhsic_config uhsic_default = {
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

static int utmip_pad_open(struct tegra_usb_phy *phy)
{
	phy->pad_clk = clk_get_sys("utmip-pad", NULL);
	if (IS_ERR(phy->pad_clk)) {
		pr_err("%s: can't get utmip pad clock\n", __func__);
		return -1;
	}

	if (phy->instance == 0) {
		phy->pad_regs = phy->regs;
	} else {
		phy->pad_regs = ioremap(TEGRA_USB_BASE, TEGRA_USB_SIZE);
		if (!phy->pad_regs) {
			pr_err("%s: can't remap usb registers\n", __func__);
			clk_put(phy->pad_clk);
			return -ENOMEM;
		}
	}
	return 0;
}

static void utmip_pad_close(struct tegra_usb_phy *phy)
{
	if (phy->instance != 0)
		iounmap(phy->pad_regs);
	clk_put(phy->pad_clk);
}

static void utmip_pad_power_on(struct tegra_usb_phy *phy)
{
	unsigned long val, flags;
	void __iomem *base = phy->pad_regs;

	clk_enable(phy->pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (utmip_pad_count++ == 0) {
		val = readl(base + UTMIP_BIAS_CFG0);
		val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
		writel(val, base + UTMIP_BIAS_CFG0);
	}

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(phy->pad_clk);
}

static int utmip_pad_power_off(struct tegra_usb_phy *phy)
{
	unsigned long val, flags;
	void __iomem *base = phy->pad_regs;

	if (!utmip_pad_count) {
		pr_err("%s: utmip pad already powered off\n", __func__);
		return -1;
	}

	clk_enable(phy->pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (--utmip_pad_count == 0) {
		val = readl(base + UTMIP_BIAS_CFG0);
		val |= UTMIP_OTGPD | UTMIP_BIASPD;
		writel(val, base + UTMIP_BIAS_CFG0);
	}

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(phy->pad_clk);

	return 0;
}

static int utmi_wait_register(void __iomem *reg, u32 mask, u32 result)
{
	unsigned long timeout = 2000;
	do {
		if ((readl(reg) & mask) == result)
			return 0;
		udelay(1);
		timeout--;
	} while (timeout);
	return -1;
}

static void utmi_phy_clk_disable(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->instance == 0) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);

		udelay(10);

		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);
	}

	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val |= USB_PORTSC1_PHCD;
		writel(val, base + USB_PORTSC1);
	}

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID, 0) < 0)
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
}

static void utmi_phy_clk_enable(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->instance == 0) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_SUSP_CLR;
		writel(val, base + USB_SUSP_CTRL);

		udelay(10);

		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_CLR;
		writel(val, base + USB_SUSP_CTRL);
	}

	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PHCD;
		writel(val, base + USB_PORTSC1);
	}

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID))
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
}

static void utmi_phy_power_on(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	int gpio_status;
	struct tegra_utmip_config *config = phy->config;

	val = readl(base + USB_SUSP_CTRL);
	val |= UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);

	if (phy->instance == 0) {
		val = readl(base + USB1_LEGACY_CTRL);
		val |= USB1_NO_LEGACY_MODE;
		writel(val, base + USB1_LEGACY_CTRL);
	}

	val = readl(base + UTMIP_TX_CFG0);
	val &= ~UTMIP_FS_PREABMLE_J;
	writel(val, base + UTMIP_TX_CFG0);

	val = readl(base + UTMIP_HSRX_CFG0);
	val &= ~(UTMIP_IDLE_WAIT(~0) | UTMIP_ELASTIC_LIMIT(~0));
	val |= UTMIP_IDLE_WAIT(config->idle_wait_delay);
	val |= UTMIP_ELASTIC_LIMIT(config->elastic_limit);
	writel(val, base + UTMIP_HSRX_CFG0);

	val = readl(base + UTMIP_HSRX_CFG1);
	val &= ~UTMIP_HS_SYNC_START_DLY(~0);
	val |= UTMIP_HS_SYNC_START_DLY(config->hssync_start_delay);
	writel(val, base + UTMIP_HSRX_CFG1);

	val = readl(base + UTMIP_DEBOUNCE_CFG0);
	val &= ~UTMIP_BIAS_DEBOUNCE_A(~0);
	val |= UTMIP_BIAS_DEBOUNCE_A(udc_debounce_table[phy->freq_sel]);
	writel(val, base + UTMIP_DEBOUNCE_CFG0);

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_SUSPEND_EXIT_ON_EDGE;
	writel(val, base + UTMIP_MISC_CFG0);

	val = readl(base + UTMIP_MISC_CFG1);
	val &= ~(UTMIP_PLL_ACTIVE_DLY_COUNT(~0) | UTMIP_PLLU_STABLE_COUNT(~0));
	val |= UTMIP_PLL_ACTIVE_DLY_COUNT(udc_delay_table[phy->freq_sel][2]) |
		UTMIP_PLLU_STABLE_COUNT(udc_delay_table[phy->freq_sel][1]);
	writel(val, base + UTMIP_MISC_CFG1);

	val = readl(base + UTMIP_PLL_CFG1);
	val &= ~(UTMIP_XTAL_FREQ_COUNT(~0) | UTMIP_PLLU_ENABLE_DLY_COUNT(~0));
	val |= UTMIP_XTAL_FREQ_COUNT(udc_delay_table[phy->freq_sel][3]) |
		UTMIP_PLLU_ENABLE_DLY_COUNT(udc_delay_table[phy->freq_sel][0]);
	writel(val, base + UTMIP_PLL_CFG1);

	if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE) {
		val = readl(base + USB_SUSP_CTRL);
		val &= ~(USB_WAKE_ON_CNNT_EN_DEV | USB_WAKE_ON_DISCON_EN_DEV);
		writel(val, base + USB_SUSP_CTRL);
	}

	utmip_pad_power_on(phy);

	val = readl(base + UTMIP_XCVR_CFG0);
	val &= ~(UTMIP_FORCE_PD_POWERDOWN | UTMIP_FORCE_PD2_POWERDOWN |
		 UTMIP_FORCE_PDZI_POWERDOWN | UTMIP_XCVR_SETUP(~0) |
		 UTMIP_XCVR_LSFSLEW(~0) | UTMIP_XCVR_LSRSLEW(~0) |
		 UTMIP_XCVR_HSSLEW_MSB(~0));
	val |= UTMIP_XCVR_SETUP(config->xcvr_setup);
	val |= UTMIP_XCVR_LSFSLEW(config->xcvr_lsfslew);
	val |= UTMIP_XCVR_LSRSLEW(config->xcvr_lsrslew);
	writel(val, base + UTMIP_XCVR_CFG0);

	val = readl(base + UTMIP_XCVR_CFG1);
	val &= ~(UTMIP_FORCE_PDDISC_POWERDOWN | UTMIP_FORCE_PDCHRP_POWERDOWN |
		 UTMIP_FORCE_PDDR_POWERDOWN | UTMIP_XCVR_TERM_RANGE_ADJ(~0));
	val |= UTMIP_XCVR_TERM_RANGE_ADJ(config->term_range_adj);
	writel(val, base + UTMIP_XCVR_CFG1);

	val = readl(base + UTMIP_BAT_CHRG_CFG0);
	val &= ~UTMIP_PD_CHRG;
	writel(val, base + UTMIP_BAT_CHRG_CFG0);

	val = readl(base + UTMIP_BIAS_CFG1);
	val &= ~UTMIP_BIAS_PDTRK_COUNT(~0);
	val |= UTMIP_BIAS_PDTRK_COUNT(0x5);
	writel(val, base + UTMIP_BIAS_CFG1);

	if (phy->instance == 0) {
		val = readl(base + UTMIP_SPARE_CFG0);
		if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE)
			val &= ~FUSE_SETUP_SEL;
		else
			val |= FUSE_SETUP_SEL;
		writel(val, base + UTMIP_SPARE_CFG0);
	}

	if (phy->instance == 2) {
		val = readl(base + USB_SUSP_CTRL);
		val |= UTMIP_PHY_ENABLE;
		writel(val, base + USB_SUSP_CTRL);
	}

	val = readl(base + USB_SUSP_CTRL);
	val &= ~UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);

	if (phy->instance == 0) {
		val = readl(base + USB1_LEGACY_CTRL);
		val &= ~USB1_VBUS_SENSE_CTL_MASK;
		val |= USB1_VBUS_SENSE_CTL_A_SESS_VLD;
		writel(val, base + USB1_LEGACY_CTRL);

		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);
		if (phy->mode == TEGRA_USB_PHY_MODE_HOST) {
			gpio_status = gpio_request(TEGRA_GPIO_PD0, "VBUS_BUS");
			if (gpio_status < 0) {
				printk(KERN_ERR "VBUS_USB1 request GPIO FAILED\n");
				WARN_ON(1);
			}
			tegra_gpio_enable(TEGRA_GPIO_PD0);
			gpio_status = gpio_direction_output(TEGRA_GPIO_PD0, 1);
			if (gpio_status < 0) {
				printk(KERN_ERR "VBUS_USB1 request GPIO DIRECTION FAILED\n");
				WARN_ON(1);
			}
			gpio_set_value(TEGRA_GPIO_PD0, 1);
			tegra_pinmux_set_tristate(TEGRA_PINGROUP_SLXK, TEGRA_TRI_NORMAL);
		}
	}

	utmi_phy_clk_enable(phy);

	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PTS(~0);
		writel(val, base + USB_PORTSC1);
	}
}

static void utmi_phy_power_off(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	utmi_phy_clk_disable(phy);

	if (phy->instance == 0 && phy->mode == TEGRA_USB_PHY_MODE_HOST) {
		gpio_free(TEGRA_GPIO_PD0);
		tegra_pinmux_set_tristate(TEGRA_PINGROUP_SLXK, TEGRA_TRI_TRISTATE);
	}

	if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE) {
		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_WAKEUP_DEBOUNCE_COUNT(~0);
		val |= USB_WAKE_ON_CNNT_EN_DEV | USB_WAKEUP_DEBOUNCE_COUNT(5);
		writel(val, base + USB_SUSP_CTRL);
	}

	val = readl(base + USB_SUSP_CTRL);
	val |= UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + UTMIP_BAT_CHRG_CFG0);
	val |= UTMIP_PD_CHRG;
	writel(val, base + UTMIP_BAT_CHRG_CFG0);

	val = readl(base + UTMIP_XCVR_CFG0);
	val |= UTMIP_FORCE_PD_POWERDOWN | UTMIP_FORCE_PD2_POWERDOWN |
	       UTMIP_FORCE_PDZI_POWERDOWN;
	writel(val, base + UTMIP_XCVR_CFG0);

	val = readl(base + UTMIP_XCVR_CFG1);
	val |= UTMIP_FORCE_PDDISC_POWERDOWN | UTMIP_FORCE_PDCHRP_POWERDOWN |
	       UTMIP_FORCE_PDDR_POWERDOWN;
	writel(val, base + UTMIP_XCVR_CFG1);

	utmip_pad_power_off(phy);
}

static void utmi_phy_preresume(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_TX_CFG0);
	val |= UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);
}

static void utmi_phy_postresume(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_TX_CFG0);
	val &= ~UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);
}

static void uhsic_phy_postresume(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + USB_TXFILLTUNING);
	if ((val & 0x00ff0000) != 0x00100000) {
		val = 0x00100000;
		writel(val, base + USB_TXFILLTUNING);
	}

}

static void utmi_phy_restore_start(struct tegra_usb_phy *phy,
				   enum tegra_usb_phy_port_speed port_speed)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_DPDM_OBSERVE_SEL(~0);
	if (port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_K;
	else
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_J;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(1);

	val = readl(base + UTMIP_MISC_CFG0);
	val |= UTMIP_DPDM_OBSERVE;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(10);
}

static void utmi_phy_restore_end(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_DPDM_OBSERVE;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(10);
}

static void ulpi_viewport_write(struct tegra_usb_phy *phy, u8 addr, u8 data)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = ULPI_RUN | ULPI_RD_RW_WRITE | ULPI_PORT(0);
	val |= ULPI_ADDR(addr) | ULPI_DATA_WR(data);
	writel(val, base + ULPI_VIEWPORT);

	if (utmi_wait_register(base + ULPI_VIEWPORT, ULPI_RUN, 0))
		pr_err("%s: timeout accessing ulpi phy\n", __func__);
}

static void ulpi_phy_power_on(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	gpio_direction_output(config->reset_gpio, 0);
	msleep(5);
	gpio_direction_output(config->reset_gpio, 1);

	clk_enable(phy->clk);
	msleep(1);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	val = 0;
	writel(val, base + ULPI_TIMING_CTRL_1);

	val |= ULPI_DATA_TRIMMER_SEL(4);
	val |= ULPI_STPDIRNXT_TRIMMER_SEL(4);
	val |= ULPI_DIR_TRIMMER_SEL(4);
	writel(val, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	val |= ULPI_DATA_TRIMMER_LOAD;
	val |= ULPI_STPDIRNXT_TRIMMER_LOAD;
	val |= ULPI_DIR_TRIMMER_LOAD;
	writel(val, base + ULPI_TIMING_CTRL_1);

	val = ULPI_WAKEUP | ULPI_RD_RW_WRITE | ULPI_PORT(0);
	writel(val, base + ULPI_VIEWPORT);

	if (utmi_wait_register(base + ULPI_VIEWPORT, ULPI_WAKEUP, 0) <0) {
		pr_err("%s: timeout waiting for ulpi phy wakeup\n", __func__);
		ulpi_phy_fail = 1;
		return;
	}

	/* Fix VbusInvalid due to floating VBUS */
	ulpi_viewport_write(phy, 0x08, 0x40);
	ulpi_viewport_write(phy, 0x0B, 0x80);

	val = readl(base + USB_PORTSC1);
	val |= USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN;
	writel(val, base + USB_PORTSC1);

	val = readl(base + USB_SUSP_CTRL);
	val |= USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
	udelay(100);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
}

static void ulpi_phy_power_off(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	/* Clear WKCN/WKDS/WKOC wake-on events that can cause the USB
	 * Controller to immediately bring the ULPI PHY out of low power
	 */
	val = readl(base + USB_PORTSC1);
	val &= ~(USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN);
	writel(val, base + USB_PORTSC1);

	gpio_direction_output(config->reset_gpio, 0);
	clk_disable(phy->clk);
}

static void null_phy_power_on(struct tegra_usb_phy *phy)
{
	const struct tegra_ulpi_trimmer default_trimmer = {0, 0, 4, 4};
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if (config->preinit)
		config->preinit();

	if (!config->trimmer)
		config->trimmer = &default_trimmer;

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	/* set timming parameters */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_SHADOW_CLK_LOOPBACK_EN;
	val |= ULPI_SHADOW_CLK_SEL;
	val |= ULPI_OUTPUT_PINMUX_BYP;
	val |= ULPI_CLKOUT_PINMUX_BYP;
	val |= ULPI_LBK_PAD_EN;
	val |= ULPI_SHADOW_CLK_DELAY(config->trimmer->shadow_clk_delay);
	val |= ULPI_CLOCK_OUT_DELAY(config->trimmer->clock_out_delay);
	val |= ULPI_LBK_PAD_E_INPUT_OR;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = 0;
	writel(val, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	/* enable null phy mode */
	val = ULPIS2S_ENA;
	val |= ULPIS2S_PLLU_MASTER_BLASTER60;
	val |= ULPIS2S_SPARE((phy->mode == TEGRA_USB_PHY_MODE_HOST) ? 3 : 1);
	writel(val, base + ULPIS2S_CTRL);

	/* select ULPI_CORE_CLK_SEL to SHADOW_CLK */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CORE_CLK_SEL;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	/* enable ULPI null clocks - can't set the trimmers before this */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_OUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	val = ULPI_DATA_TRIMMER_SEL(config->trimmer->data_trimmer);
	val |= ULPI_STPDIRNXT_TRIMMER_SEL(config->trimmer->stpdirnxt_trimmer);
	val |= ULPI_DIR_TRIMMER_SEL(4);
	writel(val, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	val |= ULPI_DATA_TRIMMER_LOAD;
	val |= ULPI_STPDIRNXT_TRIMMER_LOAD;
	val |= ULPI_DIR_TRIMMER_LOAD;
	writel(val, base + ULPI_TIMING_CTRL_1);

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_PADOUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	val = readl(base + USB_SUSP_CTRL);
	val |= USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
	udelay(100);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID))
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);

	if (config->postinit)
		config->postinit();
}

static void null_phy_power_off(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + ULPI_TIMING_CTRL_0);
	val &= ~ULPI_CLK_PADOUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
}

static void uhsic_phy_power_on(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_uhsic_config *config = &uhsic_default;
	struct tegra_ulpi_config *ulpi_config = phy->config;

	if (ulpi_config->preinit)
		ulpi_config->preinit();

	val = readl(base + UHSIC_PADS_CFG1);
	val &= ~(UHSIC_PD_BG | UHSIC_PD_TX | UHSIC_PD_TRK | UHSIC_PD_RX |
			UHSIC_PD_ZI | UHSIC_RPD_DATA | UHSIC_RPD_STROBE);
	val |= UHSIC_RX_SEL;
	writel(val, base + UHSIC_PADS_CFG1);
	udelay(2);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(30);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + UHSIC_HSRX_CFG0);
	val |= UHSIC_IDLE_WAIT(config->idle_wait_delay);
	val |= UHSIC_ELASTIC_UNDERRUN_LIMIT(config->elastic_underrun_limit);
	val |= UHSIC_ELASTIC_OVERRUN_LIMIT(config->elastic_overrun_limit);
	writel(val, base + UHSIC_HSRX_CFG0);

	val = readl(base + UHSIC_HSRX_CFG1);
	val |= UHSIC_HS_SYNC_START_DLY(config->sync_start_delay);
	writel(val, base + UHSIC_HSRX_CFG1);

	val = readl(base + UHSIC_MISC_CFG0);
	val |= UHSIC_SUSPEND_EXIT_ON_EDGE;
	writel(val, base + UHSIC_MISC_CFG0);

	val = readl(base + UHSIC_MISC_CFG1);
	val |= UHSIC_PLLU_STABLE_COUNT(uhsic_delay_table[phy->freq_sel][1]);
	writel(val, base + UHSIC_MISC_CFG1);

	val = readl(base + UHSIC_PLL_CFG1);
	val |= UHSIC_PLLU_ENABLE_DLY_COUNT(uhsic_delay_table[phy->freq_sel][0]);
	val |= UHSIC_XTAL_FREQ_COUNT(uhsic_delay_table[phy->freq_sel][3]);
	writel(val, base + UHSIC_PLL_CFG1);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~(UHSIC_RESET);
	writel(val, base + USB_SUSP_CTRL);
	udelay(2);

	val = readl(base + USB_PORTSC1);
	val &= ~USB_PORTSC1_PTS(~0);
	writel(val, base + USB_PORTSC1);

	val = readl(base + USB_TXFILLTUNING);
	if ((val & 0x00ff0000) != 0x00100000) {
		val = 0x00100000;
		writel(val, base + USB_TXFILLTUNING);
	}

	val = readl(base + USB_PORTSC1);
	val &= ~(USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN);
	writel(val, base + USB_PORTSC1);

	val = readl(base + UHSIC_PADS_CFG0);
	val &= ~(UHSIC_TX_RTUNEN);
	/* set Rtune impedance to 40 ohm */
	val |= UHSIC_TX_RTUNE(0);
	writel(val, base + UHSIC_PADS_CFG0);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
							USB_PHY_CLK_VALID)) {
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
	}
}

static void uhsic_phy_power_off(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UHSIC_PADS_CFG1);
	val &= ~UHSIC_RPU_STROBE;
	val |= UHSIC_RPD_STROBE;
	writel(val, base + UHSIC_PADS_CFG1);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(30);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~UHSIC_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

}

struct tegra_usb_phy *tegra_usb_phy_open(int instance, void __iomem *regs,
			void *config, enum tegra_usb_phy_mode phy_mode)
{
	struct tegra_usb_phy *phy;
	struct tegra_ulpi_config *ulpi_config;
	unsigned long parent_rate;
	int freq_sel;
	int err;

	phy = kmalloc(sizeof(struct tegra_usb_phy), GFP_KERNEL);
	if (!phy)
		return ERR_PTR(-ENOMEM);

	phy->instance = instance;
	phy->regs = regs;
	phy->config = config;
	phy->mode = phy_mode;

	if (!phy->config) {
		if (instance == 1) {
			pr_err("%s: ulpi/uhsic phy configuration missing", __func__);
			err = -EINVAL;
			goto err0;
		} else {
			phy->config = &utmip_default[instance];
		}
	}

	phy->pll_u = clk_get_sys(NULL, "pll_u");
	if (IS_ERR(phy->pll_u)) {
		pr_err("Can't get pll_u clock\n");
		err = PTR_ERR(phy->pll_u);
		goto err0;
	}
	clk_enable(phy->pll_u);

	parent_rate = clk_get_rate(clk_get_parent(phy->pll_u));
	for (freq_sel = 0; freq_sel < ARRAY_SIZE(udc_freq_table); freq_sel++) {
		if (udc_freq_table[freq_sel] == parent_rate)
			break;
	}
	if (freq_sel == ARRAY_SIZE(udc_freq_table)) {
		pr_err("invalid pll_u parent rate %ld\n", parent_rate);
		err = -EINVAL;
		goto err1;
	}
	phy->freq_sel = freq_sel;

	if (phy->instance == 1) {
		ulpi_config = config;

		if (ulpi_config->inf_type == TEGRA_USB_LINK_ULPI) {
			phy->clk = clk_get_sys(NULL, ulpi_config->clk);
			if (IS_ERR(phy->clk)) {
				pr_err("%s: can't get ulpi clock\n", __func__);
				err = -ENXIO;
				goto err1;
			}
			tegra_gpio_enable(ulpi_config->reset_gpio);
			gpio_request(ulpi_config->reset_gpio, "ulpi_phy_reset_b");
			gpio_direction_output(ulpi_config->reset_gpio, 0);
		}
	} else {
		err = utmip_pad_open(phy);
		if (err < 0)
			goto err1;
	}

	return phy;

err1:
	clk_disable(phy->pll_u);
	clk_put(phy->pll_u);
err0:
	kfree(phy);
	return ERR_PTR(err);
}

int tegra_usb_phy_power_on(struct tegra_usb_phy *phy)
{
	if (phy->instance == 1) {
		struct tegra_ulpi_config *ulpi_config = phy->config;
		if (ulpi_config->inf_type == TEGRA_USB_LINK_ULPI)
			ulpi_phy_power_on(phy);
		else if (ulpi_config->inf_type == TEGRA_USB_NULL_ULPI)
			null_phy_power_on(phy);
		else if (ulpi_config->inf_type == TEGRA_USB_UHSIC)
			uhsic_phy_power_on(phy);
	} else {
		utmi_phy_power_on(phy);
	}
	//add by yi-hsin for usb2 3G phy remove
	if (phy->instance == 1 && ulpi_phy_fail == 1)
		return -1;
	else
		return 0;
}

int tegra_usb_phy_power_off(struct tegra_usb_phy *phy)
{
	if (phy->instance == 1) {
		struct tegra_ulpi_config *ulpi_config = phy->config;
		if (ulpi_config->inf_type == TEGRA_USB_LINK_ULPI)
			ulpi_phy_power_off(phy);
		else if (ulpi_config->inf_type == TEGRA_USB_NULL_ULPI)
			null_phy_power_off(phy);
		else if (ulpi_config->inf_type == TEGRA_USB_UHSIC)
			uhsic_phy_power_off(phy);
	} else {
		utmi_phy_power_off(phy);
	}
	return 0;
}

int tegra_usb_phy_preresume(struct tegra_usb_phy *phy)
{
	if (phy->instance != 1)
		utmi_phy_preresume(phy);
	return 0;
}

int tegra_usb_phy_postresume(struct tegra_usb_phy *phy)
{
	struct tegra_ulpi_config *config = phy->config;

	if (phy->instance != 1)
		utmi_phy_postresume(phy);
	else if ((phy->instance == 1) &&
			(config->inf_type == TEGRA_USB_UHSIC))
		uhsic_phy_postresume(phy);
	return 0;
}

int tegra_ehci_phy_restore_start(struct tegra_usb_phy *phy,
				 enum tegra_usb_phy_port_speed port_speed)
{
	if (phy->instance != 1)
		utmi_phy_restore_start(phy, port_speed);
	return 0;
}

int tegra_ehci_phy_restore_end(struct tegra_usb_phy *phy)
{
	if (phy->instance != 1)
		utmi_phy_restore_end(phy);
	return 0;
}

int tegra_usb_phy_clk_disable(struct tegra_usb_phy *phy)
{
	if (phy->instance != 1)
		utmi_phy_clk_disable(phy);

	return 0;
}

int tegra_usb_phy_clk_enable(struct tegra_usb_phy *phy)
{
	if (phy->instance != 1)
		utmi_phy_clk_enable(phy);

	return 0;
}

int tegra_usb_phy_close(struct tegra_usb_phy *phy)
{
	if (phy->instance == 1) {
		struct tegra_ulpi_config *ulpi_config = phy->config;

		if (ulpi_config->inf_type == TEGRA_USB_LINK_ULPI)
			clk_put(phy->clk);
	} else
		utmip_pad_close(phy);
	clk_disable(phy->pll_u);
	clk_put(phy->pll_u);
	kfree(phy);
	return 0;
}

int tegra_usb_phy_bus_connect(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if ((phy->instance == 1) &&
		(config->inf_type == TEGRA_USB_UHSIC)) {

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_DETECT_SHORT_CONNECT;
		writel(val, base + UHSIC_MISC_CFG0);
		udelay(1);

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_FORCE_XCVR_MODE;
		writel(val, base + UHSIC_MISC_CFG0);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
		val |= UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		if (utmi_wait_register(base + UHSIC_STAT_CFG0, UHSIC_CONNECT_DETECT, UHSIC_CONNECT_DETECT) < 0) {
			pr_err("%s: timeout waiting for hsic connect detect\n", __func__);
			return -ETIMEDOUT;
		}

		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(2), USB_PORTSC1_LS(2)) < 0) {
			pr_err("%s: timeout waiting for dplus state\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int tegra_usb_phy_bus_reset(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if ((phy->instance == 1) &&
		(config->inf_type == TEGRA_USB_UHSIC)) {

		val = readl(base + USB_PORTSC1);
		val |= USB_PORTSC1_PTC(5);
		writel(val, base + USB_PORTSC1);
		udelay(2);

		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PTC(~0);
		writel(val, base + USB_PORTSC1);
		udelay(2);

		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(0), 0) < 0) {
			pr_err("%s: timeout waiting for SE0\n", __func__);
			return -ETIMEDOUT;
		}

		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_CCS, USB_PORTSC1_CCS) < 0) {
			pr_err("%s: timeout waiting for connection status\n", __func__);
			return -ETIMEDOUT;
		}

		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_PSPD(2), USB_PORTSC1_PSPD(2)) < 0) {
			pr_err("%s: timeout waiting hsic high speed configuration\n", __func__);
			return -ETIMEDOUT;
		}

		val = readl(base + USB_USBCMD);
		val &= ~USB_USBCMD_RS;
		writel(val, base + USB_USBCMD);

		if (utmi_wait_register(base + USB_USBSTS, USB_USBSTS_HCH, USB_USBSTS_HCH) < 0) {
			pr_err("%s: timeout waiting for stopping the controller\n", __func__);
			return -ETIMEDOUT;
		}

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPU_STROBE;
		val |= UHSIC_RPD_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		mdelay(50);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
		val |= UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		val = readl(base + USB_USBCMD);
		val |= USB_USBCMD_RS;
		writel(val, base + USB_USBCMD);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		if (utmi_wait_register(base + USB_USBCMD, USB_USBCMD_RS, USB_USBCMD_RS) < 0) {
			pr_err("%s: timeout waiting for starting the controller\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int tegra_usb_phy_bus_idle(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if ((phy->instance == 1) &&
		(config->inf_type == TEGRA_USB_UHSIC)) {

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_DETECT_SHORT_CONNECT;
		writel(val, base + UHSIC_MISC_CFG0);
		udelay(1);

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_FORCE_XCVR_MODE;
		writel(val, base + UHSIC_MISC_CFG0);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
		val |= UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);
	}
	return 0;
}

bool tegra_usb_phy_is_device_connected(struct tegra_usb_phy *phy)
{
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if ((phy->instance == 1) &&
		(config->inf_type == TEGRA_USB_UHSIC)) {
		if (!((readl(base + UHSIC_STAT_CFG0) & UHSIC_CONNECT_DETECT) == UHSIC_CONNECT_DETECT)) {
			pr_err("%s: hsic no device connection\n", __func__);
			return false;
		}
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(2), USB_PORTSC1_LS(2)) < 0) {
			pr_err("%s: timeout waiting for dplus state\n", __func__);
			return false;
		}
	}
	return true;
}

