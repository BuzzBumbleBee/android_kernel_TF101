/*
 * arch/arm/mach-tegra/board-whistler.c
 *
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_scrollwheel.h>
#include <linux/input.h>
#include <linux/tegra_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/memblock.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/audio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/tegra_das.h>

#include "board.h"
#include "clock.h"
#include "board-whistler.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTA_BASE),
		.mapbase	= TEGRA_UARTA_BASE,
		.irq		= INT_UARTA,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

#ifdef CONFIG_BCM4329_RFKILL

static struct resource whistler_bcm4329_rfkill_resources[] = {
	{
		.name	= "bcm4329_nshutdown_gpio",
		.start	= TEGRA_GPIO_PU0,
		.end	= TEGRA_GPIO_PU0,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device whistler_bcm4329_rfkill_device = {
	.name		= "bcm4329_rfkill",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(whistler_bcm4329_rfkill_resources),
	.resource	= whistler_bcm4329_rfkill_resources,
};

static noinline void __init whistler_bt_rfkill(void)
{
	platform_device_register(&whistler_bcm4329_rfkill_device);
	return;
}
#else
static inline void whistler_bt_rfkill(void) { }
#endif

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "clk_dev2",
	.inf_type = TEGRA_USB_LINK_ULPI,
};

static __initdata struct tegra_clk_init_table whistler_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uarta",	"pll_p",	216000000,	true},
	{ "uartc",	"pll_m",	600000000,	false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "pll_a",	NULL,		11289600,	true},
	{ "pll_a_out0",	NULL,		11289600,	true},
	{ "i2s1",	"pll_a_out0",	11289600,	true},
	{ "i2s2",	"pll_a_out0",	11289600,	true},
	{ "audio",	"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",	22579200,	true},
	{ "sdmmc2",	"pll_p",	25000000,	false},
	{ NULL,		NULL,		0,		0},
};

static char *usb_functions[] = { "mtp" };
static char *usb_functions_adb[] = { "mtp", "adb" };

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x7102,
		.num_functions  = ARRAY_SIZE(usb_functions),
		.functions      = usb_functions,
	},
	{
		.product_id     = 0x7100,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id              = 0x0955,
	.product_id             = 0x7100,
	.manufacturer_name      = "NVIDIA",
	.product_name           = "Whistler",
	.serial_number          = NULL,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_adb),
	.functions = usb_functions_adb,
};

static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

static struct tegra_i2c_platform_data whistler_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data whistler_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 400000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
};

static struct tegra_i2c_platform_data whistler_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static struct tegra_i2c_platform_data whistler_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
};

static struct tegra_das_platform_data tegra_das_pdata = {
	.tegra_dap_port_info_table = {
		[0] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		/* I2S1 <--> DAC1 <--> DAP1 <--> Hifi Codec */
		[1] = {
			.dac_port = tegra_das_port_i2s1,
			.codec_type = tegra_audio_codec_type_hifi,
			.device_property = {
				.num_channels = 2,
				.bits_per_sample = 16,
				.rate = 44100,
				.dac_dap_data_comm_format = dac_dap_data_format_i2s,
			},
		},
		[2] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		[3] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		[4] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
	},

	.tegra_das_con_table = {
		[0] = {
			.con_id = tegra_das_port_con_id_hifi,
			.num_entries = 4,
			.con_line = {
				[0] = {tegra_das_port_i2s1, tegra_das_port_dap1, true},
				[1] = {tegra_das_port_dap1, tegra_das_port_i2s1, false},
				[2] = {tegra_das_port_i2s2, tegra_das_port_dap4, true},
				[3] = {tegra_das_port_dap4, tegra_das_port_i2s2, false},
			},
		},
	}
};

static void whistler_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &whistler_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &whistler_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &whistler_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &whistler_dvc_platform_data;

	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}


static struct tegra_audio_platform_data tegra_audio_pdata[] = {
	/* For I2S1 */
	[0] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 44100,
		.dsp_master_clk = 44100,
		.i2s_clk_rate	= 2822400,
		.dap_clk	= "clk_dev1",
		.audio_sync_clk = "audio_2x",
		.mode		= I2S_BIT_FORMAT_I2S,
		.fifo_fmt	= I2S_FIFO_PACKED,
		.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
	},
	/* For I2S2 */
	[1] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dsp_master_clk = 8000,
		.i2s_clk_rate	= 2000000,
		.dap_clk	= "clk_dev1",
		.audio_sync_clk = "audio_2x",
		.mode		= I2S_BIT_FORMAT_DSP,
		.fifo_fmt	= I2S_FIFO_16_LSB,
		.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
	}
};



#define GPIO_SCROLL(_pinaction, _gpio, _desc)	\
{	\
	.pinaction = GPIO_SCROLLWHEEL_PIN_##_pinaction, \
	.gpio = TEGRA_GPIO_##_gpio,	\
	.desc = _desc,	\
	.active_low = 1,	\
	.debounce_interval = 2,	\
}

static struct gpio_scrollwheel_button scroll_keys[] = {
	[0] = GPIO_SCROLL(ONOFF, PR3, "sw_onoff"),
	[1] = GPIO_SCROLL(PRESS, PQ5, "sw_press"),
	[2] = GPIO_SCROLL(ROT1, PQ3, "sw_rot1"),
	[3] = GPIO_SCROLL(ROT2, PQ4, "sw_rot2"),
};

static struct gpio_scrollwheel_platform_data whistler_scroll_platform_data = {
	.buttons = scroll_keys,
	.nbuttons = ARRAY_SIZE(scroll_keys),
};

static struct platform_device whistler_scroll_device = {
	.name	= "alps-gpio-scrollwheel",
	.id	= 0,
	.dev	= {
		.platform_data	= &whistler_scroll_platform_data,
	},
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *whistler_devices[] __initdata = {
	&androidusb_device,
	&debug_uart,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&pmu_device,
	&tegra_udc_device,
	&tegra_gart_device,
	&tegra_wdt_device,
	&tegra_avp_device,
	&whistler_scroll_device,
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_das_device,
};

static struct synaptics_i2c_rmi_platform_data synaptics_pdata= {
	.flags			= SYNAPTICS_FLIP_X | SYNAPTICS_FLIP_Y | SYNAPTICS_SWAP_XY,
	.irqflags		= IRQF_TRIGGER_LOW,
};

static const struct i2c_board_info whistler_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO("synaptics-rmi-ts", 0x20),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC6),
		.platform_data	= &synaptics_pdata,
	},
};

static int __init whistler_touch_init(void)
{
	i2c_register_board_info(0, whistler_i2c_touch_info, 1);

	return 0;
}

static int __init whistler_scroll_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(scroll_keys); i++)
		tegra_gpio_enable(scroll_keys[i].gpio);

	return 0;
}


static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
		},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
		},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
};

static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
			tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
		tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
				GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci_pdata[0],
				sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	kfree(pdev->dev.platform_data);
	platform_device_unregister(pdev);
}

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.host_register = &tegra_usb_otg_host_register,
	.host_unregister = &tegra_usb_otg_host_unregister,
};

static void whistler_usb_init(void)
{
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
}

static int __init whistler_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PU4);
	return 0;
}

static const struct i2c_board_info whistler_codec_info[] = {
	{
		I2C_BOARD_INFO("wm8753", 0x1a),
	},
};

static void whistler_codec_init(void)
{
	i2c_register_board_info(4, whistler_codec_info, 1);
}
static void __init tegra_whistler_init(void)
{
	char serial[20];

	tegra_common_init();
	tegra_clk_init_from_table(whistler_clk_init_table);
	whistler_pinmux_init();
	whistler_i2c_init();
	snprintf(serial, sizeof(serial), "%llx", tegra_chip_uid());
	andusb_plat.serial_number = kstrdup(serial, GFP_KERNEL);
	tegra_i2s_device1.dev.platform_data = &tegra_audio_pdata[0];
	tegra_i2s_device2.dev.platform_data = &tegra_audio_pdata[1];
	platform_add_devices(whistler_devices, ARRAY_SIZE(whistler_devices));

	tegra_das_device.dev.platform_data = &tegra_das_pdata;

	whistler_sdhci_init();
	whistler_regulator_init();
	whistler_panel_init();
	whistler_sensors_init();
	whistler_touch_init();
	whistler_kbc_init();
	whistler_bt_rfkill();
	whistler_gps_init();
	whistler_usb_init();
	whistler_scroll_init();
	whistler_codec_init();
}

int __init tegra_whistler_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}

void __init tegra_whistler_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_128M, SZ_8M, SZ_16M);
}

MACHINE_START(WHISTLER, "whistler")
	.boot_params    = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_whistler_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_whistler_reserve,
	.timer          = &tegra_timer,
MACHINE_END
