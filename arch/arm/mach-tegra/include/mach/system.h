/*
 * arch/arm/mach-tegra/include/mach/system.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_SYSTEM_H
#define __MACH_TEGRA_SYSTEM_H

#include <mach/hardware.h>
#include <mach/iomap.h>

extern void (*tegra_reset)(char mode, const char *cmd);

static inline void arch_idle(void)
{
}

static inline void tegra_assert_system_reset(void)
{
	void __iomem *reset = IO_ADDRESS(TEGRA_PMC_BASE + 0x00);
	u32 reg;

	reg = readl(reset);
	reg |= 0x10;
	writel(reg, reset);
}

static inline void arch_reset(char mode, const char *cmd)
{
	if (tegra_reset)
		tegra_reset(mode, cmd);
	else
		tegra_assert_system_reset();

	do { } while (1);
}

#endif
