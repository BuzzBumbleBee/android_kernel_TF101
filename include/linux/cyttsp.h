/* Header file for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * include/linux/cyttsp.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */
#include <linux/input.h>

#ifndef _CYTTSP_H_
#define _CYTTSP_H_

#include <linux/input.h>

#define CY_SPI_NAME "cyttsp-spi"
#define CY_I2C_NAME "cyttsp-i2c"
/* Active Power state scanning/processing refresh interval */
#define CY_ACT_INTRVL_DFLT 0x00
/* touch timeout for the Active power */
#define CY_TCH_TMOUT_DFLT 0xFF
/* Low Power state scanning/processing refresh interval */
#define CY_LP_INTRVL_DFLT 0x0A
/*
 *defines for Gen2 (Txx2xx); Gen3 (Txx3xx)
 * use these defines to set cyttsp_platform_data.gen in board config file
 */
enum cyttsp_gen {
	CY_GEN2,
	CY_GEN3,
	CY_GEN4,
};
/*
 * Active distance in pixels for a gesture to be reported
 * if set to 0, then all gesture movements are reported
 * Valid range is 0 - 15
 */
#define CY_ACT_DIST_DFLT 8
//#define CY_ACT_DIST CY_ACT_DIST_DFLT
#define CY_ACT_DIST 1
/* max num retries to read touch data */
#define CY_NUM_RETRY 4

enum cyttsp_gest {
	CY_GEST_GRP_NONE = 0,
	CY_GEST_GRP1 =	0x10,
	CY_GEST_GRP2 = 0x20,
	CY_GEST_GRP3 = 0x40,
	CY_GEST_GRP4 = 0x80,
};

enum cyttsp_powerstate {
	CY_IDLE_STATE,
	CY_ACTIVE_STATE,
	CY_LOW_PWR_STATE,
	CY_SLEEP_STATE,
};

struct cyttsp_platform_data {
	u32 maxx;
	u32 maxy;
	u32 flags;
	enum cyttsp_gen gen;
	unsigned use_mt:1;
	unsigned use_trk_id:1;
	unsigned use_hndshk:1;
	unsigned use_timer:1;
	unsigned use_sleep:1;
	unsigned use_gestures:1;
	unsigned use_load_file:1;
	unsigned use_force_fw_update:1;
	unsigned use_virtual_keys:1;
	enum cyttsp_powerstate power_state;
	u8 gest_set;
	u8 act_intrvl;  /* Active refresh interval; ms */
	u8 tch_tmout;   /* Active touch timeout; ms */
	u8 lp_intrvl;   /* Low power refresh interval; ms */
	int (*wakeup)(void);
	int (*init)(int on_off);
	void (*mt_sync)(struct input_dev *);
	char *name;
	s16 irq_gpio;
};

#endif /* _CYTTSP_H_ */
