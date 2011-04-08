/*
 * arch/arm/mach-tegra/include/mach/clock.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef __MACH_TEGRA_CLOCK_H
#define __MACH_TEGRA_CLOCK_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <asm/clkdev.h>

#define DIV_BUS			(1 << 0)
#define DIV_U71			(1 << 1)
#define DIV_U71_FIXED		(1 << 2)
#define DIV_2			(1 << 3)
#define DIV_U16			(1 << 4)
#define PLL_FIXED		(1 << 5)
#define PLL_HAS_CPCON		(1 << 6)
#define MUX			(1 << 7)
#define PLLD			(1 << 8)
#define PERIPH_NO_RESET		(1 << 9)
#define PERIPH_NO_ENB		(1 << 10)
#define PERIPH_EMC_ENB		(1 << 11)
#define PERIPH_MANUAL_RESET	(1 << 12)
#define PLL_ALT_MISC_REG	(1 << 13)
#define PLLU			(1 << 14)
#define ENABLE_ON_INIT		(1 << 28)

#define MAX_SAME_LIMIT_SKU_IDS	16

struct clk;

struct clk_mux_sel {
	struct clk	*input;
	u32		value;
};

struct clk_pll_freq_table {
	unsigned long	input_rate;
	unsigned long	output_rate;
	u16		n;
	u16		m;
	u8		p;
	u8		cpcon;
};

struct clk_ops {
	void		(*init)(struct clk *);
	int		(*enable)(struct clk *);
	void		(*disable)(struct clk *);
	int		(*set_parent)(struct clk *, struct clk *);
	int		(*set_rate)(struct clk *, unsigned long);
	long		(*round_rate)(struct clk *, unsigned long);
	void		(*reset)(struct clk *, bool);
};

enum clk_state {
	UNINITIALIZED = 0,
	ON,
	OFF,
};

struct clk {
	/* node for master clocks list */
	struct list_head	node;		/* node for list of all clocks */
	struct dvfs 		*dvfs;
	struct clk_lookup	lookup;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*dent;
	struct dentry		*parent_dent;
#endif
	bool			set;
	struct clk_ops		*ops;
	unsigned long		dvfs_rate;
	unsigned long		rate;
	unsigned long		max_rate;
	unsigned long		min_rate;
	bool			auto_dvfs;
	bool			cansleep;
	u32			flags;
	const char		*name;

	u32			refcnt;
	enum clk_state		state;
	struct clk		*parent;
	u32			div;
	u32			mul;

	const struct clk_mux_sel	*inputs;
	u32				reg;
	u32				reg_shift;

	struct list_head		shared_bus_list;

	union {
		struct {
			unsigned int			clk_num;
		} periph;
		struct {
			unsigned long			input_min;
			unsigned long			input_max;
			unsigned long			cf_min;
			unsigned long			cf_max;
			unsigned long			vco_min;
			unsigned long			vco_max;
			const struct clk_pll_freq_table	*freq_table;
			int				lock_delay;
		} pll;
		struct {
			u32				sel;
			u32				reg_mask;
		} mux;
		struct {
			struct clk			*main;
			struct clk			*backup;
		} cpu;
		struct {
			struct list_head		node;
			bool				enabled;
			unsigned long			rate;
		} shared_bus_user;
	} u;

	struct mutex mutex;
	spinlock_t spinlock;
};

struct clk_duplicate {
	const char *name;
	struct clk_lookup lookup;
};

struct tegra_clk_init_table {
	const char *name;
	const char *parent;
	unsigned long rate;
	bool enabled;
};

struct tegra_sku_rate_limit {
	const char *clk_name;
	unsigned long max_rate;
	int sku_ids[MAX_SAME_LIMIT_SKU_IDS];
};

void tegra2_init_clocks(void);
void tegra2_periph_reset_deassert(struct clk *c);
void tegra2_periph_reset_assert(struct clk *c);
void clk_init(struct clk *clk);
struct clk *tegra_get_clock_by_name(const char *name);
unsigned long clk_measure_input_freq(void);
int clk_reparent(struct clk *c, struct clk *parent);
void tegra_clk_init_from_table(struct tegra_clk_init_table *table);
void clk_set_cansleep(struct clk *c);
unsigned long clk_get_rate_locked(struct clk *c);
void tegra2_sdmmc_tap_delay(struct clk *c, int delay);

#ifdef CONFIG_CPU_FREQ
struct cpufreq_frequency_table;

struct tegra_cpufreq_table_data {
	struct cpufreq_frequency_table *freq_table;
	int throttle_lowest_index;
	int throttle_highest_index;
};
struct tegra_cpufreq_table_data *tegra_cpufreq_table_get(void);
#endif

#endif
